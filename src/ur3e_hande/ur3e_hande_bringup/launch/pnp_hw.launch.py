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
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node


def generate_launch_description():

    # launch arguments
    use_perception = LaunchConfiguration("use_perception")
    obj_pos = LaunchConfiguration("obj_pos")
    robot_ip = LaunchConfiguration("robot_ip")
    launch_rviz = LaunchConfiguration("launch_rviz")

    declared_arguments = [
        DeclareLaunchArgument(
            "use_perception",
            default_value="true",
            description="Use perception module to detect object pose.\n"
                        "If false, you must supply obj_pos."
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
    ]

    # package directories
    ur3e_bringup_pkg = get_package_share_directory("ur3e_bringup")
    moveit_pkg = get_package_share_directory("ur3e_hande_moveit_config")
    perception_pkg = get_package_share_directory("ur3e_hande_perception")

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

    # UR3e Hardware Bringup
    ld.add_action(
        TimerAction(
            period=3.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(ur3e_bringup_pkg, "launch", "hardware.launch.py")
                    ),
                    launch_arguments={
                        "robot_ip": robot_ip,
                        "launch_rviz": launch_rviz,
                    }.items(),
                )
            ],
        )
    )

    # Perception Nodes
    ld.add_action(
        TimerAction(
            period=6.0,
            condition=IfCondition(use_perception),
            actions=[
                Node(
                    package="ur3e_hande_perception",
                    executable="pc_voxel_filter_node",
                    name="pc_voxel_filter_node",
                    output="screen",
                    parameters=[
                        {"use_sim_time": False},
                        {"input_topic": "/camera/camera/depth/color/points"},
                        {"output_topic": "/filtered_cloud"},
                        {"leaf_size": 0.005},
                        {"crop_enabled": True},
                    ],
                ),
                Node(
                    package="ur3e_hande_perception",
                    executable="pc_segmentation_node",
                    name="pc_segmentation_node",
                    output="screen",
                    parameters=[
                        {"use_sim_time": False},
                        {"input_topic": "/filtered_cloud"},
                    ],
                ),
                Node(
                    package="ur3e_hande_perception",
                    executable="obj_pose_action_server.py",
                    name="object_pose_server",
                    output="screen",
                ),
            ],
        )
    )

    # # Hand-E Gripper Bringup
    # ld.add_action(
    #     TimerAction(
    #         period=8.0,
    #         actions=[
    #             ExecuteProcess(
    #                 cmd=[
    #                     "ros2", "launch", "robotiq_hande_driver", "rs_launch.py",
    #                     "use_fake_hardware:=false",
    #                     "create_socat_tty:=true",
    #                     "tty_port:=/tmp/ttyUR"
    #                     "socat_ip_address:=192.168.77.22"
    #                     "socat_port:=54321"
    #                     "frequency_hz:=10"
    #                     "launch_rviz:=false",
    #                 ],
    #                 output="screen",
    #             )
    #         ],
    #     )
    # )
    # ld.add_action(
    #     TimerAction(
    #         period=8.0,
    #         actions=[
    #             Node(
    #                 package="robotiq_hande_ros2_driver",
    #                 executable="gripper_joint_publisher",
    #                 name="gripper_joint_publisher",
    #                 output="screen",
    #             )
    #         ],
    #     )
    # )

    # ld.add_action(
    #     TimerAction(
    #         period=10.0,64 bytes from 192.168.77.22: icmp_seq=49 ttl=64 time=0.329 ms

    #         actions=[
    #             Node(
    #                 package="robotiq_hande_ros2_driver",
    #                 executable="gripper_action_server",
    #                 name="gripper_action_server",
    #                 output="screen",
    #                 parameters=[
    #                     {"robot_ip": robot_ip},
    #                     {"p_diff_thresh": 5},
    #                     {"u_diff_thresh": 4},
    #                     {"f_diff_thresh": 5},
    #                 ],
    #             )
    #         ],
    #     )
    # )

    # MoveIt2 for Hardware
    ld.add_action(
        TimerAction(
            period=8.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(moveit_pkg, "launch", "ur3e_hande_moveit.launch.py")
                    ),
                    launch_arguments={
                        "use_sim_time": "false",
                        "launch_rviz": launch_rviz,
                    }.items(),
                )
            ],
        )
    )

    # Pick-and-Place Node
    ld.add_action(
        TimerAction(
            period=10.0,
            condition=IfCondition(use_perception),
            actions=[
                Node(
                    package="ur3e_hande_moveit_py",
                    executable="pnp_demo.py",
                    name="pnp_demo",
                    output="screen",
                )
            ],
        )
    )

    # manual obj_pos mode
    ld.add_action(
        TimerAction(
            period=12.0,
            condition=UnlessCondition(use_perception),
            actions=[
                Node(
                    package="ur3e_hande_moveit_py",
                    executable="pnp_demo.py",
                    name="pnp_demo",
                    output="screen",
                    parameters=[{"obj_pos": obj_pos}],
                )
            ],
        )
    )

    return ld
