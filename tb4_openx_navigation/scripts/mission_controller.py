#!/usr/bin/env python3
"""
MissionController
=================
Flow per area:
  1. Nav2  → drive to zone
  2. Spin  → search for ArUco marker on box side
  3. Align → visual-servo robot perpendicular & 35 cm from marker
  4. Pick  → hand-off to arm via Pick action (marker_frame = "marker")
  5. Home  → return to origin

Alignment logic (camera optical frame axes):
  position.x  →  lateral error  (+ = marker is to the RIGHT of camera centre)
  position.z  →  forward distance to marker
  orientation →  marker facing direction; we want robot X-axis anti-parallel
                 to marker Z-axis (marker Z points OUT of the tag face)

Skew convention used here
  - Extract the angle between the marker's outward normal and the camera's
    forward axis projected onto the ground plane.
  - Target: marker faces robot squarely  →  skew ≈ 0° in camera frame
    (the marker pose from ros2_aruco has Z pointing toward the camera when
     the robot faces the marker directly, giving small x/y quaternion components)
"""

import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseArray
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from tb4_openx_interfaces.action import Pick


# ─────────────────────────────────────────────
#  Tuning constants  (tweak without touching logic)
# ─────────────────────────────────────────────
APPROACH_DIST       = 0.3   # metres  – final robot-to-marker distance
LATERAL_TOL         = 0.025  # metres  – lateral centring tolerance
DIST_TOL            = 0.025  # metres  – distance tolerance
SKEW_LARGE_DEG      = 20.0   # degrees – threshold for lateral strafe correction
SKEW_FINE_DEG       = 5.0    # degrees – threshold for in-place rotation correction
SKEW_TOL_DEG        = 10.0    # degrees – "good enough" tolerance
MAX_ALIGN_ATTEMPTS  = 10      # give up after N correction cycles
SKEW_FINE_ATTEMPTS  = 5   # max fine skew corrections before accepting
SPIN_SPEED          = 0.3    # rad/s   – search spin speed
SPIN_TIMEOUT        = 14.0   # seconds – time allowed for one full 360°
LATERAL_SPEED       = 0.10   # m/s     – strafe drive speed
TURN_SPEED          = 0.45   # rad/s   – rotation speed for corrections
MARKER_FRAME        = "marker"  # TF frame name expected by Pick action server


class MissionController(Node):

    # ──────────────────────────────────────────
    def __init__(self):
        super().__init__('mission_controller')

        # Action clients
        self.nav_client  = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.pick_client = ActionClient(self, Pick,           'pick_trash')

        # Velocity publisher  (no namespace – matches topic list)
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.skew_fine_count = 0

        # ArUco vision
        self.tag_found   = False
        self.target_pose = None
        self.create_subscription(PoseArray, '/aruco_poses',
                                 self._vision_cb, 10)

        # Zone map  (add / edit zones here)
        self.zones = {
            'home':   dict(x= 0.0, y= 0.0, qz=0.0, qw=1.0),
            'area_1': dict(x= 2.0, y= 0.0, qz=0.0, qw=1.0),
            'area_2': dict(x=-2.0, y= 2.0, qz=0.0, qw=1.0),
            'area_3': dict(x= 1.0, y=-2.0, qz=0.0, qw=1.0),
        }

    # ──────────────────────────────────────────
    #  Vision callback
    # ──────────────────────────────────────────
    def _vision_cb(self, msg: PoseArray):
        if msg.poses:
            self.tag_found   = True
            self.target_pose = msg.poses[0]

    # ──────────────────────────────────────────
    #  Helpers
    # ──────────────────────────────────────────
    def _stop(self):
        self.vel_pub.publish(Twist())
        time.sleep(0.3)

    def _timed_publish(self, cmd: Twist, duration: float):
        """Publish a velocity command for `duration` seconds."""
        end = time.time() + duration
        while time.time() < end:
            self.vel_pub.publish(cmd)
            time.sleep(0.05)

    @staticmethod
    def _skew_from_pose(pose) -> float:
        """
        Return skew angle (radians) of marker relative to camera forward axis.

        ros2_aruco publishes marker pose in the camera_optical frame:
          - Z axis of the marker points OUT of the tag face (toward camera)
          - When robot faces marker squarely, marker Z ≈ camera -Z
            → quaternion is near identity with possible 180° flip

        We project the marker's facing direction onto the image XZ plane
        and return the signed angle away from the camera's forward axis (Z).
        A positive skew means marker is rotated counter-clockwise as seen
        from above (robot needs to strafe LEFT to square up).
        """
        q = pose.orientation
        # Rotate the marker's local Z-axis (0,0,1) by the quaternion
        # to get the marker normal in camera frame.
        # marker_normal = R(q) * [0,0,1]
        # Using quaternion sandwich product shortcut:
        nx = 2*(q.x*q.z + q.w*q.y)
        ny = 2*(q.y*q.z - q.w*q.x)
        nz = 1 - 2*(q.x*q.x + q.y*q.y)

        # Project onto camera XZ plane (ignore ny / vertical component)
        # skew = angle between projected normal and camera +Z
        skew = math.atan2(nx, nz)   # positive = marker tilted to robot's right
        if skew > math.pi/2:
            skew -= math.pi
        elif skew < -math.pi/2:
            skew += math.pi
        return skew

    # ──────────────────────────────────────────
    #  1. Macro-navigation (Nav2)
    # ──────────────────────────────────────────
    def go_to_pose(self, zone_name: str) -> bool:
        self.get_logger().info(f'[NAV] Navigating to {zone_name}...')
        self.nav_client.wait_for_server()

        z = self.zones[zone_name]
        goal            = NavigateToPose.Goal()
        goal.pose.header.frame_id     = 'map'
        goal.pose.header.stamp        = self.get_clock().now().to_msg()
        goal.pose.pose.position.x     = z['x']
        goal.pose.pose.position.y     = z['y']
        goal.pose.pose.orientation.z  = z['qz']
        goal.pose.pose.orientation.w  = z['qw']

        fut = self.nav_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut)
        gh = fut.result()
        if not gh.accepted:
            self.get_logger().error('[NAV] Goal rejected!')
            return False

        res_fut = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut)
        ok = res_fut.result().status == GoalStatus.STATUS_SUCCEEDED
        self.get_logger().info(f'[NAV] {"Arrived" if ok else "FAILED"}')
        return ok

    # ──────────────────────────────────────────
    #  2. Search spin
    # ──────────────────────────────────────────
    def spin_and_search(self) -> bool:
        self.get_logger().info('[SEARCH] Spinning to find ArTag...')
        self.tag_found   = False
        self.target_pose = None

        cmd = Twist()
        cmd.angular.z = SPIN_SPEED
        t0 = time.time()

        while (time.time() - t0 < SPIN_TIMEOUT) and not self.tag_found:
            self.vel_pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.05)

        self._stop()

        if not self.tag_found:
            self.get_logger().warn('[SEARCH] ArTag NOT found.')
            return False

        self.get_logger().info('[SEARCH] ArTag acquired! Centring...')

        # ── Centre marker in FOV before handing off to align ──
        for _ in range(50):   # max 5 seconds
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.target_pose is None:
                continue
            lateral_err = self.target_pose.position.x
            self.get_logger().info(f'[SEARCH] Centring lat={lateral_err:+.3f}m')
            if abs(lateral_err) < 0.02:
                self._stop()
                self.get_logger().info('[SEARCH] Marker centred ✅')
                return True
            cmd = Twist()
            cmd.angular.z = max(-0.15, min(0.15, -2.0 * lateral_err))
            self.vel_pub.publish(cmd)

        self._stop()
        return True  

    # ──────────────────────────────────────────
    #  3. Alignment state machine
    # ──────────────────────────────────────────    
    def align_to_target(self) -> bool:
        self.get_logger().info('[ALIGN] Starting visual-servo alignment...')
        attempts = 0

        while rclpy.ok() and attempts < MAX_ALIGN_ATTEMPTS:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.target_pose is None:
                self.get_logger().warn('[ALIGN] No pose – waiting...')
                continue

            pose         = self.target_pose
            lateral_err  = pose.position.x
            fwd_dist     = pose.position.z
            dist_err     = fwd_dist - APPROACH_DIST
            skew_rad     = self._skew_from_pose(pose)
            skew_deg     = math.degrees(skew_rad)

            self.get_logger().info(
                f'[ALIGN] lat={lateral_err:+.3f}m  '
                f'dist={fwd_dist:.3f}m  skew={skew_deg:+.1f}°')

            cmd = Twist()

            # ── STEP 1: Rotate toward marker first (always fix lateral first) ──
            if abs(lateral_err) > LATERAL_TOL:
                self.skew_fine_count = 0
                self.get_logger().info(
                    f'[ALIGN] Step1: Centering lateral={lateral_err:+.3f}m')
                cmd.angular.z = max(-0.25, min(0.25, -2.5 * lateral_err))
                self.vel_pub.publish(cmd)
                time.sleep(0.1)

            # ── STEP 2: Drive toward marker ─────────────────────────────────
            elif abs(dist_err) > DIST_TOL:
                self.skew_fine_count = 0
                self.get_logger().info(
                    f'[ALIGN] Step2: Driving dist_err={dist_err:+.3f}m')
                cmd.linear.x  = max(-0.08, min(0.08, 0.6 * dist_err))
                cmd.angular.z = max(-0.1,  min(0.1, -1.0 * lateral_err))
                self.vel_pub.publish(cmd)
                time.sleep(0.1)

            # ── STEP 3: Large skew → strafe correction ───────────────────────
            elif abs(skew_deg) > SKEW_LARGE_DEG:
                self.skew_fine_count = 0
                self.get_logger().warn(
                    f'[ALIGN] Step3: Large skew {skew_deg:+.1f}° → strafe')

                strafe_dist = fwd_dist * math.sin(abs(skew_rad))
                drive_time  = strafe_dist / LATERAL_SPEED
                turn_time   = (math.pi / 2.0) / TURN_SPEED
                turn_dir    = 1.0 if skew_rad > 0 else -1.0

                self.get_logger().info(
                    f'[ALIGN] Strafe {strafe_dist:.3f}m '
                    f'{"LEFT" if turn_dir < 0 else "RIGHT"}')

                # Turn 90°
                cmd.angular.z = turn_dir * TURN_SPEED
                self._timed_publish(cmd, turn_time)
                self._stop()

                # Drive laterally
                cmd = Twist()
                cmd.linear.x = LATERAL_SPEED
                self._timed_publish(cmd, drive_time)
                self._stop()

                # Turn back 90°
                cmd = Twist()
                cmd.angular.z = -turn_dir * TURN_SPEED
                self._timed_publish(cmd, turn_time)
                self._stop()

                time.sleep(3.0)
                self.target_pose = None
                attempts += 1
                continue

            # ── STEP 4: Fine skew → slow rotate in place ────────────────────
            elif abs(skew_deg) > SKEW_TOL_DEG:
                self.skew_fine_count += 1
                if self.skew_fine_count >= SKEW_FINE_ATTEMPTS:
                    self.get_logger().warn(
                        f'[ALIGN] Skew {skew_deg:.1f}° — max attempts reached, accepting')
                    self._stop()
                    return True   # ← just hand off to arm
                self.get_logger().info(
                    f'[ALIGN] Step4: Fine skew {skew_deg:+.1f}° '
                    f'({self.skew_fine_count}/{SKEW_FINE_ATTEMPTS})')
                rot_speed = math.copysign(
                    min(0.08, max(0.03, 0.004 * abs(skew_deg))), -skew_rad)
                cmd.angular.z = rot_speed
                self.vel_pub.publish(cmd)
                time.sleep(0.3)
                self._stop()
                time.sleep(0.5)
                self.target_pose = None
                continue

            # ── DONE ─────────────────────────────────────────────────────────
            else:
                self._stop()
                self.get_logger().info(
                    f'[ALIGN] ✅ Aligned!  '
                    f'lat={lateral_err:+.3f}m  '
                    f'dist={fwd_dist:.3f}m  '
                    f'skew={skew_deg:+.1f}°')
                return True

        self.get_logger().error(
            f'[ALIGN] Failed after {attempts} correction attempts')
        return False
    

    # ──────────────────────────────────────────
    #  4. Arm pick hand-off
    # ──────────────────────────────────────────
    def trigger_arm_pick(self) -> bool:
        self.get_logger().info('[PICK] Sending goal to arm...')
        self.pick_client.wait_for_server()

        goal = Pick.Goal()
        goal.marker_frame = MARKER_FRAME

        fut = self.pick_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut)
        gh = fut.result()

        if not gh.accepted:
            self.get_logger().error('[PICK] Arm rejected goal!')
            return False

        res_fut = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut)
        result = res_fut.result().result
        self.get_logger().info(
            f'[PICK] {"Success" if result.success else "FAILED"}: {result.message}')
        return result.success

    # ──────────────────────────────────────────
    #  Master mission
    # ──────────────────────────────────────────
    def execute_mission(self, target_area: str):
        self.get_logger().info(f'=== MISSION START → {target_area} ===')

        # 1. Navigate to zone
        if not self.go_to_pose(target_area):
            self.get_logger().error('[MISSION] Navigation failed – aborting.')
            return

        # 2. Search for marker
        if not self.spin_and_search():
            self.get_logger().warn('[MISSION] Marker not found – returning home.')
            self.go_to_pose('home')
            return

        # 3. Align to marker
        if not self.align_to_target():
            self.get_logger().error('[MISSION] Alignment failed – returning home.')
            self.go_to_pose('home')
            return

        # 4. Trigger arm
        self.get_logger().info('[MISSION] Handing off to arm...')
        if self.trigger_arm_pick():
            self.get_logger().info('[MISSION] Pick succeeded – returning home.')
        else:
            self.get_logger().error('[MISSION] Arm pick failed.')

        self.go_to_pose('home')
        self.get_logger().info('=== MISSION COMPLETE ===')


# ─────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = MissionController()
    node.execute_mission('area_1')   # ← change target area here
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()