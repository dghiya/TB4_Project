import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_tb4_description = get_package_share_directory('tb4_openx_description')
    xacro_file = PathJoinSubstitution([pkg_tb4_description, 'urdf', 't4_manipulator.urdf.xacro'])

    # Robot description Semantic config (Points to your overwritten SRDF)
    robot_description_semantic_path = os.path.join(
        get_package_share_directory("open_manipulator_x_moveit_config"),
        "config", "open_manipulator_x.srdf"
    )
    with open(robot_description_semantic_path, "r") as file:
        robot_description_semantic_config = file.read()
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_config}

    # kinematics yaml
    kinematics_yaml_path = os.path.join(
        get_package_share_directory("open_manipulator_x_moveit_config"),
        "config", "kinematics.yaml"
    )
    with open(kinematics_yaml_path, "r") as file:
        kinematics_yaml = yaml.safe_load(file)

    # Planning Functionality
    ompl_planning_yaml_path = os.path.join(
        get_package_share_directory("open_manipulator_x_moveit_config"),
        "config", "ompl_planning.yaml"
    )
    with open(ompl_planning_yaml_path, "r") as file:
        ompl_planning_yaml = yaml.safe_load(file)
        
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization \
            default_planner_request_adapters/FixWorkspaceBounds \
            default_planner_request_adapters/FixStartStateBounds \
            default_planner_request_adapters/FixStartStateCollision \
            default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    # Trajectory Execution
    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    # Moveit Controllers
    moveit_simple_controllers_yaml_path = os.path.join(
      get_package_share_directory("open_manipulator_x_moveit_config"),
      "config", "moveit_controllers.yaml"
    )
    with open(moveit_simple_controllers_yaml_path, "r") as file:
        moveit_simple_controllers_yaml = yaml.safe_load(file)

    moveit_controllers = {
        "moveit_simple_controller_manager": moveit_simple_controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "publish_robot_description": True,
        "publish_robot_description_semantic": True
    }

    ld = LaunchDescription()
    use_sim = LaunchConfiguration('use_sim')
    ld.add_action(DeclareLaunchArgument('use_sim', default_value='true'))

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            {'robot_description': Command([
                'xacro ', xacro_file, ' ',
                'gazebo:=ignition ', 
                'config_pkg_path:=', pkg_tb4_description])},
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            {'use_sim_time': use_sim}
        ]
    )
    ld.add_action(move_group_node)
    return ld