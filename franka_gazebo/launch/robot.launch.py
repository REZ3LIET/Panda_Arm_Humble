# Author: REZ3LIET

import os
import yaml
import time
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction, DeclareLaunchArgument
from launch.actions import RegisterEventHandler, ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder

import rclpy.logging
logger = rclpy.logging.get_logger("franka.launch")

# LOAD FILE:
def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:
        # parent of IOError, OSError *and* WindowsError where available.
        return None

def rewrite_yaml(source_file: str, root_key: str):
    if not root_key:
        return source_file

    with open(source_file, 'r') as file:
        ori_data = yaml.safe_load(file)

    updated_yaml = {root_key: ori_data}
    dst_path = f"/tmp/{time.time()}.yaml"
    with open(dst_path, 'w') as file:
        yaml.dump(updated_yaml, file)
    return dst_path

def load_yaml(package_path, file_path):

    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None

def load_robot(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    position_x = LaunchConfiguration('position_x', default='0.0')
    position_y = LaunchConfiguration('position_y', default='0.0')
    orientation_yaw = LaunchConfiguration('orientation_yaw', default='0.0')

    # Updating controller parameters
    moveit_pkg_dir = get_package_share_directory('franka_moveit')

    # Loading Robot Model
    robot_xacro = ParameterValue(
        Command([
        'xacro ', os.path.join(moveit_pkg_dir, 'config/fr3.urdf.xacro')
        ]),
        value_type=str
    )
    robot_description = {"robot_description": robot_xacro}
    print(f"Robot Desc: {robot_description}")

    # Publish TF
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        remappings=remappings,
        output="both",
        parameters=[robot_description]
    )

    # Spawn Robot in Gazebo
    spawn_robot_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "robot_description",
            "-name", "franka_arm",
            "-x", position_x,
            "-y", position_y,
            "-Y", orientation_yaw,
        ],
        output="both"
    )

    # Move Group
    # Load Controllers
    joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster', '-c', "controller_manager"],
        output="screen"
    )

    arm_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'panda_arm_controller', '-c', "controller_manager"],
        output="screen"
    )

    gripper_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'panda_ee_controller', '-c', "controller_manager"],
        output="screen"
    )

    load_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot_node,
            on_exit=[
                joint_state_controller,
                arm_controller,
                gripper_controller
            ]
        )
    )

    moveit_pkg = "franka_moveit"
    moveit_config = (
        MoveItConfigsBuilder("fr3", package_name=moveit_pkg)
        .robot_description(file_path="config/fr3.urdf.xacro")
        .planning_pipelines("pilz_industrial_motion_planner")
        .joint_limits(file_path="config/joint_limits.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .robot_description_semantic(file_path="config/fr3.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .pilz_cartesian_limits(file_path="config/pilz_cartesian_limits.yaml")
        .to_moveit_configs()
    )

    planning_scene_parameters={
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "publish_robot_description": True,
        "publish_robot_description_semantic": True
    }

    # ompl_planning_pipeline_config = {
    #     "ompl": {
    #         "planning_plugin": "ompl_interface/OMPLPlanner",
    #         "request_adapters": "default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/ResolveConstraintFrames default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints",
    #         "start_state_max_bounds_error": 0.1,
    #     },
    # }

    # ompl_planning_yaml = load_yaml(
    #     get_package_share_directory(moveit_pkg), "config/ompl_planning.yaml"
    # )

    # ompl_planning_pipeline_config["ompl"].update(ompl_planning_yaml)

    # pilz_pipeline = {
    #         'pilz_industrial_motion_planner': {
    #         'planning_plugin': 'pilz_industrial_motion_planner/CommandPlanner', 
    #         # 'request_adapters': 'default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints', 
    #         "request_adapters": """ """,
    #         "start_state_max_bounds_error": 0.1,
    #         'default_planner_config': 'PTP', 
    #         'capabilities': 'pilz_industrial_motion_planner/MoveGroupSequenceAction pilz_industrial_motion_planner/MoveGroupSequenceService'
    #     }
    # }

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            planning_scene_parameters,
            # ompl_planning_pipeline_config,
            # pilz_pipeline,
            {"use_sim_time": use_sim_time}
        ]
    )

    load_move_group = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gripper_controller,
            on_exit=[move_group_node]
        )
    )

    return [
        DeclareLaunchArgument(
            name="use_sim_time",
            default_value=use_sim_time,
            description="Use simulator time",
            choices=["True", "False"]
        ),
        DeclareLaunchArgument(
            name="position_x",
            default_value=position_x,
            description="X position to spawn the robot"
        ),
        DeclareLaunchArgument(
            name="position_y",
            default_value=position_y,
            description="Y position to spawn the robot"
        ),
        DeclareLaunchArgument(
            name="orientation_yaw",
            default_value=orientation_yaw,
            description="Yaw angle to spawn robot"
        ),

        robot_state_publisher,
        spawn_robot_node,
        # load_controllers,
        # load_move_group
    ]

def generate_launch_description():
    gz_sim = LaunchConfiguration('gz_sim', default='True')

    # Loading Gazebo
    world = os.path.join(get_package_share_directory("franka_gazebo"), "world/empty_world.sdf")
    ign_gazebo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory("ros_gz_sim"), "launch"), "/gz_sim.launch.py"]
        ),
        launch_arguments={'gz_args': [world, ' -r -v1']}.items(),
        condition=IfCondition(gz_sim)
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name="gz_sim",
            default_value='True',
            description="Use gazebo simulation",
            choices=["True", "False"]
        ),
        
        ign_gazebo_node,
        OpaqueFunction(function=load_robot)
    ])