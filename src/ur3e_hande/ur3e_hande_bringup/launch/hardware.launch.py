#!/usr/bin/env python3
"""
Unified hardware bringup for the UR3e + Hand-E gripper + RealSense + Perception + MoveIt + PnP.

Components launched:
  - RealSense RGB-D camera
  - UR3e ROS2 driver (hardware.launch.py)
  - Hand-E gripper joint publisher (state feedback)
  - Hand-E gripper action server (command execution)
  - Perception pipeline (voxel --> segmentation --> object metadata)
  - Object pose action server
  - MoveIt2 (hardware controllers)
  - Pick-and-place node (perception or manual position)
"""

import os
from os import path
from os.path import expanduser

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import AnyLaunchDescriptionSource

from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
    PythonExpression,
    EnvironmentVariable
)
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():

    # launch arguments
    use_perception = LaunchConfiguration("use_perception")
    obj_pos = LaunchConfiguration("obj_pos")
    robot_ip = LaunchConfiguration("robot_ip")
    launch_rviz = LaunchConfiguration("launch_rviz")
    launch_ur_rviz = LaunchConfiguration("launch_ur_rviz")
    use_moveit = LaunchConfiguration("use_moveit")
    ur_type = LaunchConfiguration("ur_type")
    use_tool_communication = LaunchConfiguration("use_tool_communication")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    pnp = LaunchConfiguration("pnp")
    print_metrics = LaunchConfiguration("print_metrics")
    max_acc_scale = LaunchConfiguration("max_acc_scale")
    max_vel_scale = LaunchConfiguration("max_vel_scale")
    goal_pos_tol = LaunchConfiguration("goal_pos_tol")
    goal_ori_tol = LaunchConfiguration("goal_ori_tol")
    rviz_config = LaunchConfiguration("rviz_config")
    use_sim_time = LaunchConfiguration("use_sim_time")
    log_level = LaunchConfiguration("log_level")
    controllers_file = LaunchConfiguration("controllers_file")
    description_launchfile = LaunchConfiguration("description_launchfile")
    pregrasp_z = LaunchConfiguration("pregrasp_z")
    place_offset_y = LaunchConfiguration("place_offset_y")

    declared_arguments = [
        DeclareLaunchArgument(
            "use_perception",
            default_value="false",
            description="Use perception module to detect object pose."
        ),
        DeclareLaunchArgument(
            "ur_type",
            description="Type/series of used UR robot.",
            choices=[
                "ur3",
                "ur3e",
                "ur5",
                "ur5e",
                "ur7e",
                "ur10",
                "ur10e",
                "ur12e",
                "ur16e",
                "ur15",
                "ur20",
                "ur30",
            ],
            default_value="ur3e",
        ),
        DeclareLaunchArgument(
            "controllers_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("ur3e_hande_description"), "config", "controllers.yaml"]
            ),
            description="YAML file with the combined controllers configuration.",
        ),
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="false",
            description="Start robot with mock hardware mirroring command to its states.",
        ),
        DeclareLaunchArgument(
            "tf_prefix",
            default_value="",
            description="TF prefix.",
        ),
        DeclareLaunchArgument(
            "use_tool_communication",
            default_value="false",
            description="Start robot with mock hardware mirroring command to its states.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulated time.",
        ),
        DeclareLaunchArgument(
            "use_moveit",
            default_value="false",
            description="Whether to start MoveIt.",
        ),
         DeclareLaunchArgument(
            "max_acc_scale",
            default_value="0.25",
            description="Acceleration scaling for MoveIt2 trajectories.",
        ),
        DeclareLaunchArgument(
            "log_level",
            default_value="warn",
            description="ROS 2 log level.",
        ),
        DeclareLaunchArgument(
            "max_vel_scale",
            default_value="0.25",
            description="Velocity scaling for MoveIt2 trajectories.",
        ),
        DeclareLaunchArgument(
            "goal_pos_tol",
            default_value="0.003",
            description="Position tolerance for MoveIt2 goals (meters).",
        ),
        DeclareLaunchArgument(
            "goal_ori_tol",
            default_value="0.01",
            description="Orientation tolerance for MoveIt2 goals (radians).",
        ),
        DeclareLaunchArgument(
            "description_package",
            default_value="ur3e_hande_description",
            description="Package with robot description.",
        ),
        DeclareLaunchArgument(
            "description_filepath",
            default_value=path.join("urdf", "ur3e_hande_hw.urdf.xacro"),
            description="Path to URDF/Xacro file, relative to share of `description_package`.",
        ),
        DeclareLaunchArgument(
            "pregrasp_z",
            default_value="0.11",
            description="Extra Z-offset added to the pre-grasp pose (meters).",
        ),
        DeclareLaunchArgument(
            "place_offset_y",
            default_value="0.0",
            description="Lateral offset along Y for placement (meters).",
        ),
        DeclareLaunchArgument(
            "pnp",
            default_value="false", # safe default to avoid accidental execution
            description="Whether to launch the pick-and-place demo client.",
        ),
        DeclareLaunchArgument(
            "print_metrics",
            default_value="false",
            description="Whether to print pick-and-place metrics to console.",
        ),
        DeclareLaunchArgument(
            "obj_pos",
            default_value="0.30 0.50 0.10",
            description="Manual (x y z) object position (used only when use_perception=false)."
        ),
        DeclareLaunchArgument(
            "robot_ip",
            default_value="192.168.77.22",
            description="IP address of the UR3e robot controller."
        ),
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",   
            description="Launch RViz or not."
        ),
        DeclareLaunchArgument(
            "launch_ur_rviz",
            default_value="false",   
            description="Launch UR-only RViz or not. Not useful here since we need both descriptions."
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=path.join(
                get_package_share_directory("ur3e_hande_description"),
                "rviz",
                "view_robot_hw.rviz",
            ),
            description="Path to configuration for RViz2.",
        ),
        DeclareLaunchArgument(
            "description_launchfile",
            default_value=PathJoinSubstitution(
                [FindPackageShare("ur3e_hande_description"), "launch", "ur3e_hande_rsp.launch.py"]
            ),
            description="Launchfile (absolute path) providing the description. "
            "The launchfile has to start a robot_state_publisher node that "
            "publishes the description topic.",
        )
    ]

    # package directories
    ur3e_hardware_bringup_pkg = get_package_share_directory("ur_robot_driver")
    moveit_pkg = get_package_share_directory("ur3e_hande_moveit_config")

    ld = LaunchDescription(declared_arguments)

    # RealSense Bringup
    ld.add_action(
        TimerAction(
            period=0.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        "ros2", "launch", "realsense2_camera", "rs_launch.py",
                        "depth_module.depth_profile:=640x480x30",
                        "rgb_camera.color_profile:=640x480x30",
                        "pointcloud.enable:=true",
                        "align_depth.enable:=true",
                        "pointcloud.ordered_pc:=true",
                        "enable_infra:=true",
                        "enable_infra1:=true",
                        "enable_infra2:=true",
                        "depth_module.infra_profile:=640x480x30",
                    ],
                    output="screen",
                )
            ],
        )
    )

    ld.add_action(
        TimerAction(
            period=2.0,
            actions=[

            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(description_launchfile),
                launch_arguments={
                    "robot_ip": robot_ip,
                    "ur_type": ur_type,
             }.items(),
                )
            ]
        )
    )

    # UR3e Hardware Bringup
    ld.add_action(
        TimerAction(
            period=3.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(ur3e_hardware_bringup_pkg, "launch", "ur_control.launch.py")
                    ),
                    launch_arguments={
                        "ur_type": ur_type,
                        "robot_ip": robot_ip,
                        "launch_rviz": launch_ur_rviz,
                        "use_tool_communication": use_tool_communication,
                        "use_mock_hardware": use_mock_hardware,
                        "controllers_file": controllers_file,
                        "description_launchfile": description_launchfile,
                    }.items(),
                )
            ],
        )
        )

    # rviz2
    ld.add_action(
        TimerAction(
            period=8.0,
            actions=[
                Node(
                    package="rviz2",
                    condition=IfCondition(
                        PythonExpression(["'", launch_rviz, "' == 'true' and not ('", use_moveit, "' == 'true')"])
                    ),
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
                )
                ])
    )

    # Perception Nodes
    ld.add_action(
        TimerAction(
            period=9.0,
            condition=IfCondition(use_perception),
            actions=[
                Node(
                    package="ur3e_hande_perception",
                    executable="pc_voxel_filter_node",
                    name="pc_voxel_filter_node",
                    output="screen",
                    parameters=[
                    {"use_sim_time": use_sim_time},
                    {"input_topic": "/camera/camera/depth/color/points"},
                    {"output_topic": "/filtered_cloud"},
                    {"leaf_size": 0.005},
                    {"crop_enabled": True},
                    {"stop_after_first": False}, # keep processing point clouds since cloud is sparser in real life
                    {"crop_bounds": [0.0, 1.22, 0, 0.51, 0.00, 1.5]},  # xmin, xmax, ymin, ymax, zmin, zmax; Point cloud bounds: x[0.00, 1.22], y[0.00, 0.51], z[0.00, 1.50]

                    ],
                ),
                Node(
                    package="ur3e_hande_perception",
                    executable="pc_segmentation_node",
                    name="pc_segmentation_node",
                    output="screen",
                    parameters=[
                        {"use_sim_time": use_sim_time},
                        {"input_topic": "/filtered_cloud"},
                        {"base_frame": "base_link"},
                        {"camera_frame": "camera_depth_optical_frame"},
                        {"stop_after_first_pub": False}, # keep publishing segmentation results
                        ],
                ),
                Node(
                    package="ur3e_hande_perception",
                    executable="obj_pose_action_server",
                    name="object_pose_server",
                    output="screen",
                ),
            ],
        )
    )

    # MoveIt2 for Hardware
    ld.add_action(
        TimerAction(
            period=12.0,
            condition=IfCondition(use_moveit),
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(moveit_pkg, "launch", "ur3e_hande_moveit.launch.py")
                    ),
                    launch_arguments={
                        "use_sim_time": use_sim_time,
                        "log_level": log_level,
                        "launch_rviz": launch_rviz,
                    }.items(),
                )
            ],
        )
    )

    # Pick-and-Place Node
    ld.add_action(
        TimerAction(
            period=14.0,
            condition=IfCondition(pnp),
            actions=[
                Node(
                    package="ur3e_hande_moveit_py",
                    executable="pnp_demo",
                    name="pnp_demo",
                    output="screen",
                    parameters=[
                    {"use_sim_time": True},
                    {"max_vel_scale": ParameterValue(max_vel_scale, value_type=float)},
                    {"max_acc_scale": ParameterValue(max_acc_scale, value_type=float)},
                    {"goal_pos_tol": ParameterValue(goal_pos_tol, value_type=float)},
                    {"goal_ori_tol": ParameterValue(goal_ori_tol, value_type=float)},
                    {"pregrasp_z": ParameterValue(pregrasp_z, value_type=float)},
                    {"place_offset_y": ParameterValue(place_offset_y, value_type=float)},
                    {"print_metrics": ParameterValue(print_metrics, value_type=bool)},
                    # {"obj_pos": obj_pos}, # If obj_pos is omitted, PickAndPlace will query GetTargetObjPose
                ],
                )
            ],
        )
    )

    return ld
