#!/usr/bin/env -S ros2 launch
from os import path
from os.path import expanduser
import os
from typing import List

from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, AppendEnvironmentVariable, OpaqueFunction, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
    PythonExpression,
    EnvironmentVariable
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ros_gz_bridge.actions import RosGzBridge
from ament_index_python import get_package_share_directory

def generate_launch_description() -> LaunchDescription:
    # Declare all launch arguments
    declared_arguments = generate_declared_arguments()

    # Launch configurations
    description_package = LaunchConfiguration("description_package")
    description_filepath = LaunchConfiguration("description_filepath")
    default_world = LaunchConfiguration("default_world")
    world_file = LaunchConfiguration("world_file")
    robot_model = LaunchConfiguration("robot_model")
    use_sim_time = LaunchConfiguration("use_sim_time")
    gz_verbosity = LaunchConfiguration("gz_verbosity")
    log_level = LaunchConfiguration("log_level")
    rviz_config = LaunchConfiguration("rviz_config")
    launch_rviz = LaunchConfiguration("launch_rviz")
    use_teleop =  LaunchConfiguration("use_teleop")

    # ros_gz_bridge
    bridge_name = LaunchConfiguration('bridge_name')
    config_file = LaunchConfiguration('config_file')

    # add models from package share so Gazebo Sim can find package:// URIs
    description_pkg_share = get_package_share_directory("ur3e_hande_description")
    gazebo_pkg_share = get_package_share_directory("ur3e_hande_gz")
    local_model_path = os.path.join(description_pkg_share, "meshes")

    # Nodes
    aruco_marker_id = LaunchConfiguration("aruco_marker_id")

    # add gz-specific models folder
    gz_models_path = os.path.join(gazebo_pkg_share, "models")
    extra_paths = [local_model_path, gz_models_path]
    gz_sim_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')

    if gz_sim_resource_path:
        gz_sim_resource_path = ":".join(extra_paths) + ":" + gz_sim_resource_path
    else:
        gz_sim_resource_path = ":".join(extra_paths)

    # ros2 controllers config
    controllers_config = PathJoinSubstitution([
                FindPackageShare("ur3e_hande_description"),
                "config",
                "controllers.yaml",
            ])

    # URDF via xacro
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([FindPackageShare(description_package), description_filepath])
    ])

    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # Include Gazebo Harmonic 
    launch_descriptions = [
        SetEnvironmentVariable(
            name="GZ_SIM_RESOURCE_PATH",
            value=gz_sim_resource_path
        ),

        ExecuteProcess(cmd=['echo', EnvironmentVariable('GZ_SIM_RESOURCE_PATH')], output='screen'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("ros_gz_sim"),
                    "launch",
                    "gz_sim.launch.py",
                ])
            ),
            launch_arguments=[("gz_args", 
                               [
                                world_file,
                                TextSubstitution(text=" -v "),
                                gz_verbosity,
                                TextSubstitution(text=" -r "),
                                TextSubstitution(text=" --physics-engine gz-physics-bullet-featherstone-plugin") #gz-physics-bullet-featherstone-plugin (collision broken with featherstone and dartsim plugin); bullet: no mimic joint support
                            ]
                            )]
                            ),
    ]
    
    # Nodes
    nodes = [
        # Robot State Publisher
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            arguments=["--ros-args", "--log-level", log_level],
            parameters=[
                robot_description,
                {"publish_frequency": 50.0, "frame_prefix": "", "use_sim_time": use_sim_time},
            ],
        ),
        # rviz2
        Node(
            package="rviz2",
            condition=IfCondition(launch_rviz),
            executable="rviz2",
            output="log",
            arguments=[
                "--display-config",
                rviz_config,
                "--ros-args",
                "--log-level",
                log_level,
            ],
            parameters=[{"use_sim_time": use_sim_time}],
        ),

        # Load controllers using controller manager
        # TimerAction(
        #     period=2.0,
        #     actions=[
        #         Node(
        #         package="controller_manager",
        #         executable="ros2_control_node",
        #         parameters=[controllers_config],
        #         output="both",
        #         remappings=[
        #             ("~/robot_description", "/robot_description"),
        #         ],
        #         )]),

        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=["joint_state_broadcaster", "--param-file", controllers_config, "--controller-manager", "/controller_manager"],
                    output="screen"
                ),
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=["scaled_joint_trajectory_controller", "--param-file", controllers_config, "--controller-manager", "/controller_manager"],
                    output="screen"
                ),
            ],
        ),
        # Node(
        #     package="controller_manager",
        #     executable="spawner",
        #     arguments=["forward_velocity_controller", "--param-file", controllers_config, "--controller-manager", "/controller_manager"],
        #     output="screen",
        # ),
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=["gripper_action_controller", "--param-file", controllers_config, "--controller-manager", "/controller_manager"],
                    output="screen"
                ),
            ],
        ),
        # Switch to forward_velocity_controller if use_teleop is specified
        ExecuteProcess(
            condition=IfCondition(use_teleop),
            cmd=[
                'ros2', 'control', 'switch_controllers', '--deactivate', 'scaled_joint_trajectory_controller', '--activate',
                '--forward_velocity_controller', '-c', '/controller_manager'
            ],
            shell=True,
            output='log'
            ),
        # Spawn into Gazebo
        Node(
            package="ros_gz_sim",
            executable="create",
            output="screen",
            arguments=[
                "-topic", "robot_description",
                "-name", robot_model,
                "--ros-args", 
                "--log-level", log_level,
            ],
            parameters=[{"use_sim_time": use_sim_time}],
        ),
        # ros_gz_bridge
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="ros_gz_bridge",
            output="log",
            arguments=[
                # GzSim->ROS2 Bridge
                "/clock" + "@rosgraph_msgs/msg/Clock" + "[gz.msgs.Clock",
                # "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
                # "/joint_states" + "@sensor_msgs/msg/JointState" + "[gz.msgs.Model",
                # "/tf" + "@tf2_msgs/msg/TFMessage" + "[gz.msgs.Pose_V",
                "/rgbd_camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
                "/rgbd_camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
                "/rgbd_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
                "/rgbd_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
                "--ros-args",
                "--log-level",
                log_level,
            ],
            parameters=[{"use_sim_time": use_sim_time}],
        ),

        # ArUco detection
        # Node(
        #     package="aruco_pose_estimation",
        #     executable="detect_aruco.py",
        #     parameters=[{
        #         "aruco_marker_id": aruco_marker_id,
        #         "use_sim_time": use_sim_time,
        #     }],
        #     output="screen",
        # ),

        # Custom marker-to-arm motion node
        # Node(
        #     package="aruco_pose_estimation",
        #     condition=IfCondition(LaunchConfiguration("move_to_aruco")),
        #     parameters=[{"use_sim_time": use_sim_time}],
        #     executable="move_to_aruco.py",
        #     output="screen",
        # )
    ]

    return LaunchDescription(declared_arguments + launch_descriptions + nodes)


def generate_declared_arguments() -> List[DeclareLaunchArgument]:
    return [
        DeclareLaunchArgument(
            "description_package",
            default_value="ur3e_hande_description",
            description="Package with robot description.",
        ),
        DeclareLaunchArgument(
            "description_filepath",
            default_value=path.join("urdf", "ur3e_hande.urdf.xacro"),
            description="Path to URDF/Xacro file, relative to share of `description_package`.",
        ),
        DeclareLaunchArgument(
            "world_to_spawn",
            choices=['basic', 'bookshelf', 'bin', 'smallbox'],
            default_value='bookshelf',
            description="World to load in Gazebo."
        ),
        DeclareLaunchArgument(
            "world_file",
            default_value=PathJoinSubstitution([
                FindPackageShare('ur3e_hande_gz'),
                'worlds',
                PythonExpression(["'", LaunchConfiguration('world_to_spawn'), ".sdf'"])
            ]),
            description="World SDF file to load in Gazebo.",
        ),
        DeclareLaunchArgument(
            "default_world",
            default_value='default.sdf',
            description="World SDF file to load in Gazebo.",
        ),
        DeclareLaunchArgument(
            "robot_model",
            default_value="ur3e_hande",
            description="Name for spawned robot model.",
        ),
        DeclareLaunchArgument(
            "objects_to_spawn",
            default_value="",
            description="Dictionary of object model names to spawn and their spawn positions \n" \
            "Note: named objects must exist locally at a path appended \n" \
            "to the GZ_SIM_RESOURCE_PATH).",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulated time.",
        ),
        DeclareLaunchArgument(
            "gz_verbosity",
            default_value="3",
            description="Gazebo verbosity (0~4).",
        ),
        DeclareLaunchArgument(
            "log_level",
            default_value="warn",
            description="ROS 2 log level.",
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=path.join(
                get_package_share_directory("ur3e_hande_description"),
                "rviz",
                "view_robot_tfs_no_pc.rviz",
            ),
            description="Path to configuration for RViz2.",
        ),
        DeclareLaunchArgument(
            "bridge_name", 
            default_value="ur3e_hande_gz_bridge",
            description="YAML config file"
        ),
        DeclareLaunchArgument(
            "config_file",
            default_value=path.join(
                get_package_share_directory("ur3e_hande_gz"),
                "config",
                "bridge.yaml",
            ),
            description='ros_gz_bridge configuration file.'
        ),
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Launch RViz2 with the robot model and TFs."
        ),
        DeclareLaunchArgument(
            "aruco_marker_id",
            default_value="80",
            description="ID of the ArUco marker to detect."
        ),
        DeclareLaunchArgument(
            "move_to_aruco",
            default_value="false",
            description="Move the robot to the ArUco marker."
        ),
        DeclareLaunchArgument(
            "use_teleop",
            default_value="false",
            description="Whether to use a teleop interface to control the manipulator"        
            )
    ]
