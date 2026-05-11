#!/usr/bin/env python3
"""
Gemini Robotics-ER 1.6 perception node — one-shot detect + latch + republish.

Subscribes to RGB + depth + camera_info, sends the RGB frame to Gemini
Robotics-ER, parses the returned normalized [y, x] coordinate,
back-projects through the synchronized depth image, and estimates
orientation via depth-plane-fit (z-axis) + Canny-edge PCA (in-plane
x-axis).  Publishes camera -> target_frame TF (default: marker_gemini).

Latching adapter (default ON):
  After `min_consecutive_detections` agreeing API calls, the marker pose
  is anchored in the `map_frame` (default: odom).  A fast republish timer
  continuously re-emits the camera -> marker TF by chaining through the
  live robot localization.  No further API calls are made.

  A circuit breaker stops API calls if the anchor frame is unreachable
  (e.g. map_frame=map but AMCL isn't running), capping quota waste at
  `max_failed_latch_attempts` (default 5).

Orientation note:
  z-axis (surface normal) is observed from depth.  In-plane rotation is
  estimated from the dominant edge direction in the RGB patch — this is
  more accurate than an arbitrary convention, but still approximate
  compared to ArUco's solvePnP with known corner geometry.

API reference:
    https://ai.google.dev/gemini-api/docs/robotics-overview

Environment:
    GOOGLE_API_KEY  — required; free-tier key from
                      https://aistudio.google.com/apikey

Install (outside colcon, one-time):
    pip install google-genai opencv-python numpy
"""

import collections
import json
import os
import re
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Empty
from geometry_msgs.msg import TransformStamped
import tf2_ros
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from cv_bridge import CvBridge
import message_filters

# Gemini SDK — imported lazily so the node can start and print a
# helpful error if the package is missing.
try:
    from google import genai
    from google.genai import types
    _GENAI_AVAILABLE = True
except ImportError:
    _GENAI_AVAILABLE = False


# ---------------------------------------------------------------------------
#  Prompt template
#
#  Gemini Robotics-ER returns spatial coordinates in normalized [y, x]
#  format where both values are in the range 0–1000.
#  See: https://ai.google.dev/gemini-api/docs/robotics-overview
# ---------------------------------------------------------------------------
_PROMPT_TEMPLATE = (
    "Detect the {target} in this image. "
    "Return a JSON array of detections. Each detection must have a "
    '"point" field with [y, x] in normalized 0-1000 coordinates and a '
    '"label" field. If not visible, return an empty array [].'
)


class GeminiBodyTFNode(Node):
    """Gemini Robotics-ER body-camera TF publisher."""

    def __init__(self):
        super().__init__("gemini_body_tf_node")

        # ---- Parameters ------------------------------------------------
        self.declare_parameter("target_label", "artag")
        self.declare_parameter("camera_frame", "oakd_rgb_camera_optical_frame")
        self.declare_parameter("target_frame", "marker_gemini")
        self.declare_parameter("model_name", "gemini-robotics-er-1.6-preview")
        self.declare_parameter("confidence_threshold", 0.4)
        self.declare_parameter("depth_patch_size", 5)
        # Orientation patch must be larger than depth patch — we fit a 3D plane
        # to all valid back-projected points in this window to estimate the
        # marker's surface normal.
        self.declare_parameter("orientation_patch_size", 21)
        self.declare_parameter("min_plane_points", 12)
        self.declare_parameter("publish_rate_limit_hz", 0.05)
        # use_sim_time is declared automatically by ROS 2 when passed from launch
        self.declare_parameter("max_stamp_age_sec", 0.15)
        # Topic defaults — no namespace prefix in the new tb4_project repo
        self.declare_parameter("rgb_topic", "/oakd/rgb/preview/image_raw")
        self.declare_parameter("depth_topic", "/oakd/depth/image_raw")
        self.declare_parameter("camera_info_topic", "/oakd/rgb/preview/camera_info")
        # ---- Latching adapter ------------------------------------------
        # When latch_in_map=True (production):
        #   1. The first successful Gemini detection composes the camera->marker
        #      pose with the existing map->camera TF (from AMCL/odom) to get a
        #      static map->marker pose. That pose is cached in the node.
        #   2. The Gemini API timer is disabled (no further calls).
        #   3. A fast republish timer continuously emits camera->marker by
        #      chaining inv(map->camera_now) ∘ map->marker_cached, preserving
        #      the slice contract that ArUco also satisfies.
        # When latch_in_map=False (debug):
        #   Every API tick produces a fresh camera->marker (Phase 1 behavior).
        self.declare_parameter("latch_in_map", True)
        # Default to "odom" because it is published by Gazebo's diff-drive
        # controller out of the box. "map" requires AMCL / localization and
        # will silently fail if you only run gazebo_sim.launch.py.
        # Pass map_frame:=map once navigate.launch.py is running.
        self.declare_parameter("map_frame", "odom")
        self.declare_parameter("republish_rate_hz", 30.0)
        self.declare_parameter("redetect_topic", "/gemini/redetect")
        self.declare_parameter("tf_lookup_timeout_sec", 0.5)
        # Robustness: how many detections must agree before we lock the
        # marker pose into the map frame. Each detection is one API call,
        # so this directly trades quota for reliability.
        self.declare_parameter("min_consecutive_detections", 3)
        # Max translation disagreement (meters) among the candidate
        # detections before we accept them. Beyond this we discard the
        # batch and keep accumulating fresh ones.
        self.declare_parameter("latch_position_tolerance_m", 0.10)
        # Plane-fit outlier rejection: keep only depth pixels whose depth
        # is within `mad_outlier_threshold * 1.4826 * MAD` of the median
        # depth in the orientation patch. Filters background pixels that
        # leak into the patch when the marker is small in the image.
        self.declare_parameter("mad_outlier_threshold", 2.5)
        # Republish health: after this many consecutive map->camera
        # lookup failures, escalate from DEBUG to WARN.
        self.declare_parameter("republish_fail_warn_threshold", 30)
        # Circuit breaker: if the Gemini API call succeeds but _try_latch()
        # fails the TF lookup this many consecutive times, STOP calling the
        # API entirely and log an ERROR. Protects the daily quota when the
        # anchor frame doesn't exist (e.g. map without AMCL).
        # Reset by publishing on the redetect topic.
        self.declare_parameter("max_failed_latch_attempts", 5)

        self._target_label = self.get_parameter("target_label").value
        self._camera_frame = self.get_parameter("camera_frame").value
        self._target_frame = self.get_parameter("target_frame").value
        self._model_name = self.get_parameter("model_name").value
        self._conf_thresh = self.get_parameter("confidence_threshold").value
        self._patch_size = self.get_parameter("depth_patch_size").value
        self._orient_patch = self.get_parameter("orientation_patch_size").value
        self._min_plane_pts = self.get_parameter("min_plane_points").value
        self._rate_hz = self.get_parameter("publish_rate_limit_hz").value
        self._max_stamp_age = self.get_parameter("max_stamp_age_sec").value

        rgb_topic = self.get_parameter("rgb_topic").value
        depth_topic = self.get_parameter("depth_topic").value
        cam_info_topic = self.get_parameter("camera_info_topic").value

        self._latch_in_map = bool(self.get_parameter("latch_in_map").value)
        self._map_frame = self.get_parameter("map_frame").value
        self._republish_hz = float(self.get_parameter("republish_rate_hz").value)
        redetect_topic = self.get_parameter("redetect_topic").value
        self._tf_timeout = float(self.get_parameter("tf_lookup_timeout_sec").value)
        self._min_consec = int(self.get_parameter("min_consecutive_detections").value)
        self._latch_pos_tol = float(self.get_parameter("latch_position_tolerance_m").value)
        self._mad_thresh = float(self.get_parameter("mad_outlier_threshold").value)
        self._republish_fail_warn = int(
            self.get_parameter("republish_fail_warn_threshold").value)
        self._max_failed_latch = int(
            self.get_parameter("max_failed_latch_attempts").value)

        # ---- Validate Gemini SDK ---------------------------------------
        if not _GENAI_AVAILABLE:
            self.get_logger().fatal(
                "google-genai not installed. Run:\n"
                "  pip install google-genai"
            )
            raise RuntimeError("Missing google-genai")

        api_key = os.environ.get("GOOGLE_API_KEY", "")
        if not api_key:
            self.get_logger().fatal(
                "GOOGLE_API_KEY environment variable not set. "
                "Get a free key at https://aistudio.google.com/apikey"
            )
            raise RuntimeError("GOOGLE_API_KEY not set")

        self._client = genai.Client(api_key=api_key)
        self.get_logger().info(
            f"Gemini client ready: {self._model_name}  "
            f"target={self._target_label}  "
            f"frame={self._camera_frame}->{self._target_frame}"
        )

        # ---- ROS plumbing ----------------------------------------------
        self._bridge = CvBridge()
        self._tf_broadcaster = TransformBroadcaster(self)

        # tf2 buffer/listener — used for the latching adapter to look up
        # map -> camera while composing/decomposing the marker pose.
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # Latch state: T_map_marker (4x4) once locked, plus a flag.
        self._latched = False
        self._T_map_marker = None  # 4x4 numpy or None
        self._latch_lock = threading.Lock()
        # Candidate buffer: collect `min_consecutive_detections` agreeing
        # detections before committing to a latch. Each entry is the 4x4
        # T_map_marker computed at that detection's stamp.
        self._candidates = collections.deque(maxlen=max(self._min_consec, 1))
        # Republish health monitoring
        self._republish_fail_count = 0
        # Circuit breaker: counts consecutive API calls whose detection
        # succeeded but _try_latch() failed the TF lookup.
        self._failed_latch_count = 0
        self._circuit_broken = False

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ---- Synchronized subscribers ----------------------------------
        # message_filters.ApproximateTimeSynchronizer ensures the RGB and
        # depth frames we pair were captured at (nearly) the same instant,
        # preventing timing-skew from contaminating the TF comparison.
        self._sub_rgb = message_filters.Subscriber(
            self, Image, rgb_topic, qos_profile=sensor_qos)
        self._sub_depth = message_filters.Subscriber(
            self, Image, depth_topic, qos_profile=sensor_qos)
        self._sub_info = message_filters.Subscriber(
            self, CameraInfo, cam_info_topic, qos_profile=sensor_qos)

        self._sync = message_filters.ApproximateTimeSynchronizer(
            [self._sub_rgb, self._sub_depth, self._sub_info],
            queue_size=5,
            slop=0.05,  # 50 ms tolerance
        )
        self._sync.registerCallback(self._synced_cb)

        # We still rate-limit Gemini calls with a timer + cached synced set
        self._synced_data = None       # (rgb_msg, depth_msg, info_msg)
        self._data_lock = threading.Lock()

        period = 1.0 / max(self._rate_hz, 0.01)
        self._timer = self.create_timer(period, self._timer_cb)

        # Guard against overlapping Gemini calls (API can be slow)
        self._call_in_progress = False

        # ---- Latching adapter: republish timer + redetect subscriber ----
        if self._latch_in_map:
            rep_period = 1.0 / max(self._republish_hz, 0.1)
            self._republish_timer = self.create_timer(
                rep_period, self._republish_cb)
            self._redetect_sub = self.create_subscription(
                Empty, redetect_topic, self._redetect_cb, 1)
            self.get_logger().info(
                f"Latching mode ON. After first detection, will republish "
                f"{self._camera_frame}->{self._target_frame} at "
                f"{self._republish_hz:.1f} Hz from cached {self._map_frame}-frame "
                f"pose. Send std_msgs/Empty to '{redetect_topic}' to refresh."
            )
        else:
            self.get_logger().info(
                "Latching mode OFF — every API tick publishes a fresh detection "
                "(debug behavior, eats your daily quota)."
            )

        self.get_logger().info(
            f"Subscribed (synced): rgb={rgb_topic}  depth={depth_topic}  "
            f"info={cam_info_topic}  api_rate={self._rate_hz:.2f} Hz"
        )

    # ---- Synchronized callback: cache the matched triplet ---------------
    def _synced_cb(self, rgb_msg: Image, depth_msg: Image, info_msg: CameraInfo):
        with self._data_lock:
            self._synced_data = (rgb_msg, depth_msg, info_msg)

    # ---- Timer: grab latest synced data, call Gemini, publish TF --------
    def _timer_cb(self):
        if self._call_in_progress:
            return  # previous API call still running

        # In latching mode, skip API calls once we have a map-frame anchor.
        # The republish timer keeps camera->marker fresh from cache.
        if self._latch_in_map:
            with self._latch_lock:
                if self._latched:
                    return
            # Circuit breaker: stop burning quota when the anchor frame is
            # unreachable (e.g. map_frame="map" but AMCL isn't running).
            if self._circuit_broken:
                return

        with self._data_lock:
            data = self._synced_data
            self._synced_data = None  # consume it

        if data is None:
            self.get_logger().debug(
                "Waiting for synchronized rgb/depth/camera_info...",
                throttle_duration_sec=5.0,
            )
            return

        rgb_msg, depth_msg, info_msg = data

        # Reject stale frames (e.g. if timer fires long after last sync)
        now = self.get_clock().now()
        stamp = rclpy.time.Time.from_msg(rgb_msg.header.stamp)
        age = (now - stamp).nanoseconds * 1e-9
        if age > self._max_stamp_age:
            self.get_logger().debug(
                f"Stale frame ({age:.3f}s old), skipping",
                throttle_duration_sec=5.0,
            )
            return

        # Run the blocking API call in a background thread so we don't
        # starve the ROS executor.
        self._call_in_progress = True
        thread = threading.Thread(
            target=self._run_gemini,
            args=(rgb_msg, depth_msg, info_msg),
            daemon=True,
        )
        thread.start()

    # ---- Gemini inference + TF publish (runs in thread) ----------------
    def _run_gemini(self, rgb_msg: Image, depth_msg: Image, info_msg: CameraInfo):
        try:
            self._process(rgb_msg, depth_msg, info_msg)
        except Exception as e:
            self.get_logger().warn(
                f"Gemini call failed: {e}", throttle_duration_sec=5.0
            )
        finally:
            self._call_in_progress = False

    def _process(self, rgb_msg: Image, depth_msg: Image, info_msg: CameraInfo):
        # 1. Convert RGB to JPEG bytes for Gemini
        cv_rgb = self._bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
        h, w = cv_rgb.shape[:2]
        success, jpeg_buf = cv2.imencode(".jpg", cv_rgb)
        if not success:
            self.get_logger().warn("Failed to JPEG-encode frame")
            return
        jpeg_bytes = jpeg_buf.tobytes()

        # 2. Build prompt
        prompt = _PROMPT_TEMPLATE.format(target=self._target_label)

        # 3. Call Gemini Robotics-ER
        t0 = time.monotonic()
        response = self._client.models.generate_content(
            model=self._model_name,
            contents=[
                types.Part.from_bytes(
                    data=jpeg_bytes,
                    mime_type="image/jpeg",
                ),
                prompt,
            ],
            config=types.GenerateContentConfig(
                temperature=0.0,
                thinking_config=types.ThinkingConfig(thinking_budget=0),
            ),
        )
        dt = time.monotonic() - t0
        self.get_logger().info(
            f"Gemini response in {dt:.2f}s", throttle_duration_sec=3.0
        )

        # 4. Parse JSON response
        #    Expected: [{"point": [y, x], "label": "artag"}]
        #    Coordinates are normalized 0-1000.
        #    Gemini often wraps JSON in ```json ... ``` fences — strip them.
        text = response.text.strip()
        text = self._strip_markdown_fences(text)
        try:
            detections = json.loads(text)
        except json.JSONDecodeError:
            self.get_logger().warn(
                f"Gemini returned unparseable text: {text[:200]}",
                throttle_duration_sec=5.0,
            )
            return

        # Normalize: if Gemini returned a single dict, wrap it in a list
        if isinstance(detections, dict):
            detections = [detections]

        if not isinstance(detections, list) or len(detections) == 0:
            self.get_logger().info(
                f"Target '{self._target_label}' not found by Gemini",
                throttle_duration_sec=5.0,
            )
            return

        # Find the first detection whose label matches our target
        det = None
        for d in detections:
            label = d.get("label", "")
            if self._target_label.lower() in label.lower():
                det = d
                break
        if det is None:
            # No label match — fall back to first detection
            det = detections[0]

        # Accept "point": [y, x]  OR  "pixel_u"/"pixel_v" (raw pixels)
        point = det.get("point")
        if point is not None and len(point) == 2:
            # Gemini Robotics-ER returns [y, x] normalized to 0-1000
            norm_y, norm_x = float(point[0]), float(point[1])
        elif "pixel_u" in det and "pixel_v" in det:
            # Fallback: raw pixel coords (convert to normalized)
            raw_u = float(det["pixel_u"])
            raw_v = float(det["pixel_v"])
            norm_x = raw_u * 1000.0 / max(w - 1, 1)
            norm_y = raw_v * 1000.0 / max(h - 1, 1)
        else:
            self.get_logger().warn(
                f"Malformed detection (no point or pixel_u/v): {det}",
                throttle_duration_sec=5.0,
            )
            return

        # Confidence: use 1.0 if missing (Robotics-ER doesn't always return it)
        confidence = float(det.get("confidence", 1.0))
        if confidence < self._conf_thresh:
            self.get_logger().info(
                f"Low confidence {confidence:.2f} < {self._conf_thresh}",
                throttle_duration_sec=5.0,
            )
            return

        if not (0 <= norm_x <= 1000 and 0 <= norm_y <= 1000):
            self.get_logger().warn(
                f"Point out of [0,1000] range: y={norm_y}, x={norm_x}"
            )
            return

        # Convert normalized coords to pixel coords
        pixel_u = int(round(norm_x * (w - 1) / 1000.0))
        pixel_v = int(round(norm_y * (h - 1) / 1000.0))

        pixel_u = max(0, min(w - 1, pixel_u))
        pixel_v = max(0, min(h - 1, pixel_v))

        # 5. Depth lookup (median over small patch) from SYNCED depth
        cv_depth = self._bridge.imgmsg_to_cv2(
            depth_msg, desired_encoding="passthrough")
        depth_val = self._get_median_depth(cv_depth, pixel_u, pixel_v, h, w)

        if depth_val is None or depth_val <= 0.0 or np.isnan(depth_val) or np.isinf(depth_val):
            self.get_logger().warn(
                f"Invalid depth {depth_val} at pixel ({pixel_u},{pixel_v})",
                throttle_duration_sec=5.0,
            )
            return

        # 6. Back-project using pinhole model
        fx = info_msg.k[0]
        fy = info_msg.k[4]
        cx = info_msg.k[2]
        cy = info_msg.k[5]

        if fx == 0.0 or fy == 0.0:
            self.get_logger().warn(
                "Camera intrinsics not yet available (fx or fy is 0)",
                throttle_duration_sec=5.0,
            )
            return

        z = float(depth_val)
        x = (pixel_u - cx) * z / fx
        y = (pixel_v - cy) * z / fy

        # 7. Estimate orientation via plane fit (z-axis) + edge PCA (x-axis).
        #    Falls back to identity if the surface is degenerate.
        quat = self._estimate_orientation(
            cv_depth, pixel_u, pixel_v, h, w, fx, fy, cx, cy,
            cv_rgb=cv_rgb,
        )
        if quat is None:
            qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0
            ori_status = "identity-fallback"
        else:
            qx, qy, qz, qw = quat
            ori_status = "plane+edge"

        # 8. Publish TF
        t = TransformStamped()
        t.header.stamp = rgb_msg.header.stamp  # use frame timestamp, not wall
        t.header.frame_id = self._camera_frame
        t.child_frame_id = self._target_frame
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self._tf_broadcaster.sendTransform(t)
        self.get_logger().info(
            f"Published {self._camera_frame}->{self._target_frame}  "
            f"xyz=({x:.3f},{y:.3f},{z:.3f})  "
            f"quat=({qx:+.3f},{qy:+.3f},{qz:+.3f},{qw:+.3f}) [{ori_status}]  "
            f"px=({pixel_u},{pixel_v})",
            throttle_duration_sec=2.0,
        )

        # 9. Latching adapter — freeze the marker pose in the map frame.
        if self._latch_in_map:
            self._try_latch(rgb_msg.header.stamp, x, y, z, qx, qy, qz, qw)

    # ------------------------------------------------------------------
    #  Latching adapter
    # ------------------------------------------------------------------
    def _try_latch(self, stamp, x, y, z, qx, qy, qz, qw):
        """Accumulate detections; latch only when N agree.

        One bad detection (noisy depth, partial occlusion, label hallucination)
        could otherwise lock the system to a wrong pose. We require
        `min_consecutive_detections` candidates whose translations all sit
        within `latch_position_tolerance_m` of each other before committing.
        """
        try:
            T_map_camera_msg = self._tf_buffer.lookup_transform(
                self._map_frame,
                self._camera_frame,
                stamp,
                timeout=rclpy.duration.Duration(seconds=self._tf_timeout),
            )
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self._failed_latch_count += 1
            if self._failed_latch_count >= self._max_failed_latch:
                self._circuit_broken = True
                self.get_logger().error(
                    f"CIRCUIT BREAKER TRIPPED after {self._failed_latch_count} "
                    f"consecutive latch failures. The anchor frame "
                    f"'{self._map_frame}' is not reachable from "
                    f"'{self._camera_frame}'. Stopping all API calls to "
                    f"protect your daily quota. "
                    f"Fix: either start localization (navigate.launch.py) "
                    f"or set map_frame:=odom, then publish Empty to the "
                    f"redetect topic to retry."
                )
            else:
                self.get_logger().warn(
                    f"Latch lookup failed ({self._failed_latch_count}/"
                    f"{self._max_failed_latch} before circuit break): {e}",
                    throttle_duration_sec=5.0,
                )
            return

        # TF lookup succeeded — reset the failure counter.
        self._failed_latch_count = 0

        T_map_camera = self._transform_to_matrix(T_map_camera_msg.transform)
        T_camera_marker = self._make_matrix(x, y, z, qx, qy, qz, qw)
        T_map_marker = T_map_camera @ T_camera_marker

        # ---- Multi-detection agreement check ---------------------------
        with self._latch_lock:
            self._candidates.append(T_map_marker)
            n = len(self._candidates)
            need = self._min_consec

            if n < need:
                self.get_logger().info(
                    f"Latch candidate {n}/{need}  "
                    f"map_xyz=({T_map_marker[0,3]:.3f},"
                    f"{T_map_marker[1,3]:.3f},{T_map_marker[2,3]:.3f})  "
                    f"(need {need - n} more in agreement)"
                )
                return

            # We have `need` candidates — check that they agree
            translations = np.array([T[:3, 3] for T in self._candidates])
            centroid = translations.mean(axis=0)
            max_dev = float(
                np.max(np.linalg.norm(translations - centroid, axis=1))
            )

            if max_dev > self._latch_pos_tol:
                # Disagreement too large — drop the OLDEST candidate and
                # keep accumulating. This way one stale detection doesn't
                # poison the rest of the batch indefinitely.
                dropped = self._candidates.popleft()
                self.get_logger().warn(
                    f"Latch candidates disagree: max deviation "
                    f"{max_dev:.3f} m > tolerance {self._latch_pos_tol:.3f} m. "
                    f"Dropping oldest candidate (xyz="
                    f"{dropped[0,3]:.2f},{dropped[1,3]:.2f},"
                    f"{dropped[2,3]:.2f}) and waiting for another."
                )
                return

            # All candidates agree → average them and commit
            T_avg = self._average_matrices(list(self._candidates))
            self._T_map_marker = T_avg
            self._latched = True
            cand_count = n
            spread = max_dev

        self.get_logger().info(
            f"LATCHED marker pose in '{self._map_frame}' frame from "
            f"{cand_count} agreeing detections (spread={spread*100:.1f} cm). "
            f"map_xyz=({T_avg[0,3]:.3f},{T_avg[1,3]:.3f},{T_avg[2,3]:.3f}). "
            f"Switching to {self._republish_hz:.1f} Hz republish from cache. "
            f"No more Gemini calls until /gemini/redetect."
        )

    @staticmethod
    def _average_matrices(matrices):
        """Average a list of 4x4 transforms; re-orthogonalize the rotation.

        Element-wise mean is incorrect for rotation matrices in general
        (closure not preserved), so we project the averaged 3x3 block back
        onto SO(3) via SVD: R_avg ≈ U V^T from SVD(R_mean).
        """
        M = np.mean(np.stack(matrices, axis=0), axis=0)
        R = M[:3, :3]
        U, _, Vt = np.linalg.svd(R)
        R_ortho = U @ Vt
        if np.linalg.det(R_ortho) < 0:
            Vt[-1, :] *= -1
            R_ortho = U @ Vt
        out = np.eye(4)
        out[:3, :3] = R_ortho
        out[:3, 3] = M[:3, 3]
        return out

    def _republish_cb(self):
        """Republish camera->marker by chaining inv(map->camera) ∘ map->marker."""
        with self._latch_lock:
            if not self._latched or self._T_map_marker is None:
                return
            T_map_marker = self._T_map_marker.copy()

        # Use latest available map -> camera (rclpy.time.Time() == 0 means
        # "give me the most recent transform you have").
        try:
            T_map_camera_msg = self._tf_buffer.lookup_transform(
                self._map_frame,
                self._camera_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05),
            )
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self._republish_fail_count += 1
            # Most missed ticks are transient (clock drift, AMCL not
            # publishing for 100ms). After a sustained outage, escalate
            # so the operator knows the TF chain has actually broken.
            if self._republish_fail_count == self._republish_fail_warn:
                self.get_logger().warn(
                    f"map→camera lookup has failed "
                    f"{self._republish_fail_count} consecutive ticks "
                    f"(~{self._republish_fail_count / max(self._republish_hz, 1.0):.1f}s) — "
                    f"check that '{self._map_frame}' is being published. "
                    f"Last error: {e}"
                )
            else:
                self.get_logger().debug(
                    "map→camera lookup failed in republish, skipping tick",
                    throttle_duration_sec=2.0,
                )
            return

        # Recovery — reset and announce
        if self._republish_fail_count >= self._republish_fail_warn:
            self.get_logger().info(
                f"map→camera lookup recovered after "
                f"{self._republish_fail_count} failed ticks."
            )
        self._republish_fail_count = 0

        T_map_camera = self._transform_to_matrix(T_map_camera_msg.transform)
        try:
            T_camera_map = np.linalg.inv(T_map_camera)
        except np.linalg.LinAlgError:
            return
        T_camera_marker = T_camera_map @ T_map_marker

        # Use the lookup's stamp — that's when this map->camera was valid.
        msg = self._matrix_to_transform_stamped(
            T_camera_marker,
            frame_id=self._camera_frame,
            child_frame_id=self._target_frame,
            stamp=T_map_camera_msg.header.stamp,
        )
        self._tf_broadcaster.sendTransform(msg)

    def _redetect_cb(self, _msg: Empty):
        """Clear latch + candidate buffer + circuit breaker."""
        with self._latch_lock:
            was_latched = self._latched
            self._latched = False
            self._T_map_marker = None
            self._candidates.clear()
        self._republish_fail_count = 0
        self._failed_latch_count = 0
        was_broken = self._circuit_broken
        self._circuit_broken = False
        if was_broken:
            self.get_logger().info(
                "Redetect requested — circuit breaker RESET. "
                "API calls will resume on the next timer tick."
            )
        elif was_latched:
            self.get_logger().info(
                "Redetect requested — latch cleared. "
                "Next Gemini call will refresh the anchor."
            )
        else:
            self.get_logger().info(
                "Redetect requested — candidates cleared. "
                "Will start fresh accumulation."
            )

    @staticmethod
    def _strip_markdown_fences(text: str) -> str:
        """Strip ```json ... ``` or ``` ... ``` fences from Gemini output."""
        # Match ```json\n...\n``` or ```\n...\n```
        m = re.search(r'```(?:json)?\s*\n?(.*?)\n?\s*```', text, re.DOTALL)
        if m:
            return m.group(1).strip()
        return text

    def _get_median_depth(self, depth_img, u, v, h, w):
        """Return median depth in a small patch around (u, v)."""
        half = self._patch_size // 2
        v_min = max(0, v - half)
        v_max = min(h, v + half + 1)
        u_min = max(0, u - half)
        u_max = min(w, u + half + 1)

        patch = depth_img[v_min:v_max, u_min:u_max].astype(np.float64)
        valid = patch[(patch > 0) & np.isfinite(patch)]
        if valid.size == 0:
            return None
        return float(np.median(valid))

    # ------------------------------------------------------------------
    #  Orientation: surface-normal-from-depth + in-plane from edges
    # ------------------------------------------------------------------
    #  Two-stage orientation estimation:
    #
    #  Stage 1 — z-axis (surface normal):
    #    Back-project valid depth pixels in a window, MAD-filter outliers,
    #    fit a plane via SVD.  Smallest singular vector = normal.
    #
    #  Stage 2 — in-plane rotation (x-axis):
    #    Run Canny on the RGB patch around the detection, then PCA on the
    #    edge pixel coordinates.  The first principal component gives the
    #    dominant edge direction in the image.  That 2D direction is
    #    back-projected onto the 3D marker plane to define the marker
    #    x-axis, recovering the rotation about the normal that pure
    #    depth analysis cannot observe.
    #
    #    If Canny + PCA fails (too few edges, texture-less surface) we
    #    fall back to projecting camera +x onto the plane (same as before).
    #
    #  Marker frame convention (matches ArUco):
    #    z-axis  = surface normal pointing OUT of the marker face
    #    x-axis  = dominant edge direction on the marker surface
    #    y-axis  = z × x  (right-handed)
    # ------------------------------------------------------------------
    def _estimate_orientation(self, depth_img, u, v, h, w, fx, fy, cx, cy,
                              cv_rgb=None):
        """Return (qx, qy, qz, qw) or None on failure."""
        half = self._orient_patch // 2
        v_min = max(0, v - half)
        v_max = min(h, v + half + 1)
        u_min = max(0, u - half)
        u_max = min(w, u + half + 1)

        # Vectorized back-projection of all valid depths in the patch
        patch = depth_img[v_min:v_max, u_min:u_max].astype(np.float64)
        vv, uu = np.mgrid[v_min:v_max, u_min:u_max]
        mask = (patch > 0) & np.isfinite(patch)
        if int(mask.sum()) < self._min_plane_pts:
            return None

        z = patch[mask]
        x = (uu[mask] - cx) * z / fx
        y = (vv[mask] - cy) * z / fy
        pts = np.column_stack([x, y, z])  # (N, 3)

        # ---- MAD outlier rejection ---------------------------------
        z_med = float(np.median(pts[:, 2]))
        mad = float(np.median(np.abs(pts[:, 2] - z_med)))
        if mad > 1e-6:
            cutoff = self._mad_thresh * 1.4826 * mad
            inliers = np.abs(pts[:, 2] - z_med) <= cutoff
            if int(inliers.sum()) >= self._min_plane_pts:
                pts = pts[inliers]

        # ---- Stage 1: plane fit for z-axis (surface normal) --------
        centroid = pts.mean(axis=0)
        centered = pts - centroid
        try:
            _, _, vh = np.linalg.svd(centered, full_matrices=False)
        except np.linalg.LinAlgError:
            return None
        normal = vh[-1]
        n_norm = np.linalg.norm(normal)
        if n_norm < 1e-6:
            return None
        normal = normal / n_norm

        # Force normal toward camera (negative z in optical frame)
        if normal[2] > 0:
            normal = -normal
        z_axis = normal

        # ---- Stage 2: edge-based in-plane x-axis -------------------
        x_axis = self._edge_based_x_axis(
            cv_rgb, u, v, h, w, fx, fy, cx, cy, z_axis
        )

        if x_axis is None:
            # Fallback: project camera +x onto the marker plane
            cam_x = np.array([1.0, 0.0, 0.0])
            x_axis = cam_x - np.dot(cam_x, z_axis) * z_axis
            if np.linalg.norm(x_axis) < 1e-6:
                cam_y = np.array([0.0, 1.0, 0.0])
                x_axis = cam_y - np.dot(cam_y, z_axis) * z_axis
                if np.linalg.norm(x_axis) < 1e-6:
                    return None
        x_axis = x_axis / np.linalg.norm(x_axis)

        y_axis = np.cross(z_axis, x_axis)

        R = np.column_stack([x_axis, y_axis, z_axis])
        return self._matrix_to_quat(R)

    def _edge_based_x_axis(self, cv_rgb, u, v, h, w, fx, fy, cx, cy, z_axis):
        """Extract the dominant edge direction in the RGB patch and project
        it onto the 3D marker plane.

        Returns a 3D unit vector (the marker x-axis in camera_optical_frame)
        or None if edges are insufficient.

        Algorithm:
          1. Extract a grayscale patch around the detection.
          2. Canny edge detection.
          3. PCA on the (row, col) coordinates of edge pixels → PC1 is the
             dominant edge direction in pixel space.
          4. Convert that 2D direction to a 3D ray using the pinhole model
             (delta_u, delta_v → delta_x, delta_y at the marker's depth).
          5. Project the 3D direction onto the marker plane (subtract the
             component along z_axis).
        """
        if cv_rgb is None:
            return None

        # Use a slightly larger patch for edge detection (more context)
        edge_half = self._orient_patch // 2 + 4
        ev_min = max(0, v - edge_half)
        ev_max = min(h, v + edge_half + 1)
        eu_min = max(0, u - edge_half)
        eu_max = min(w, u + edge_half + 1)

        rgb_patch = cv_rgb[ev_min:ev_max, eu_min:eu_max]
        if rgb_patch.size == 0:
            return None

        gray = cv2.cvtColor(rgb_patch, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150)

        # Edge pixel coordinates (row, col within patch)
        edge_coords = np.argwhere(edges > 0)  # (N, 2) — [row, col]
        if edge_coords.shape[0] < 10:
            return None

        # PCA: dominant direction in image space
        edge_f = edge_coords.astype(np.float64)
        edge_centered = edge_f - edge_f.mean(axis=0)
        try:
            _, _, vh_edge = np.linalg.svd(edge_centered, full_matrices=False)
        except np.linalg.LinAlgError:
            return None

        # PC1 in (row, col) order = (delta_v, delta_u) in image coords
        pc1 = vh_edge[0]
        delta_v_px, delta_u_px = float(pc1[0]), float(pc1[1])

        # Convert pixel direction to 3D direction at the marker's depth
        # (Using the derivative of the pinhole model at the detection point)
        delta_x_3d = delta_u_px / fx
        delta_y_3d = delta_v_px / fy
        dir_3d = np.array([delta_x_3d, delta_y_3d, 0.0])

        # Project onto the marker plane (remove component along normal)
        dir_3d = dir_3d - np.dot(dir_3d, z_axis) * z_axis

        if np.linalg.norm(dir_3d) < 1e-8:
            return None
        return dir_3d / np.linalg.norm(dir_3d)

    @staticmethod
    def _matrix_to_quat(R):
        """3x3 rotation matrix -> (x, y, z, w) quaternion (Shepperd's method)."""
        tr = R[0, 0] + R[1, 1] + R[2, 2]
        if tr > 0.0:
            s = 2.0 * np.sqrt(tr + 1.0)
            qw = 0.25 * s
            qx = (R[2, 1] - R[1, 2]) / s
            qy = (R[0, 2] - R[2, 0]) / s
            qz = (R[1, 0] - R[0, 1]) / s
        elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
            s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            qw = (R[2, 1] - R[1, 2]) / s
            qx = 0.25 * s
            qy = (R[0, 1] + R[1, 0]) / s
            qz = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            qw = (R[0, 2] - R[2, 0]) / s
            qx = (R[0, 1] + R[1, 0]) / s
            qy = 0.25 * s
            qz = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            qw = (R[1, 0] - R[0, 1]) / s
            qx = (R[0, 2] + R[2, 0]) / s
            qy = (R[1, 2] + R[2, 1]) / s
            qz = 0.25 * s
        # Normalize to be safe
        n = np.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if n < 1e-12:
            return 0.0, 0.0, 0.0, 1.0
        return float(qx / n), float(qy / n), float(qz / n), float(qw / n)

    # ------------------------------------------------------------------
    #  4x4 homogeneous-transform helpers (used by the latching adapter)
    # ------------------------------------------------------------------
    @staticmethod
    def _quat_to_matrix(qx, qy, qz, qw):
        """Quaternion (x,y,z,w) -> 3x3 rotation matrix."""
        n = qx * qx + qy * qy + qz * qz + qw * qw
        if n < 1e-12:
            return np.eye(3)
        s = 2.0 / n
        xx = qx * qx * s
        yy = qy * qy * s
        zz = qz * qz * s
        xy = qx * qy * s
        xz = qx * qz * s
        yz = qy * qz * s
        wx = qw * qx * s
        wy = qw * qy * s
        wz = qw * qz * s
        return np.array([
            [1.0 - (yy + zz), xy - wz,         xz + wy],
            [xy + wz,         1.0 - (xx + zz), yz - wx],
            [xz - wy,         yz + wx,         1.0 - (xx + yy)],
        ])

    @staticmethod
    def _make_matrix(x, y, z, qx, qy, qz, qw):
        """(translation, quaternion) -> 4x4 homogeneous matrix."""
        T = np.eye(4)
        T[:3, :3] = GeminiBodyTFNode._quat_to_matrix(qx, qy, qz, qw)
        T[0, 3] = x
        T[1, 3] = y
        T[2, 3] = z
        return T

    @staticmethod
    def _transform_to_matrix(transform):
        """geometry_msgs/Transform -> 4x4 matrix."""
        return GeminiBodyTFNode._make_matrix(
            transform.translation.x,
            transform.translation.y,
            transform.translation.z,
            transform.rotation.x,
            transform.rotation.y,
            transform.rotation.z,
            transform.rotation.w,
        )

    @staticmethod
    def _matrix_to_transform_stamped(T, frame_id, child_frame_id, stamp):
        """4x4 matrix -> geometry_msgs/TransformStamped."""
        msg = TransformStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.child_frame_id = child_frame_id
        msg.transform.translation.x = float(T[0, 3])
        msg.transform.translation.y = float(T[1, 3])
        msg.transform.translation.z = float(T[2, 3])
        qx, qy, qz, qw = GeminiBodyTFNode._matrix_to_quat(T[:3, :3])
        msg.transform.rotation.x = qx
        msg.transform.rotation.y = qy
        msg.transform.rotation.z = qz
        msg.transform.rotation.w = qw
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = GeminiBodyTFNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
