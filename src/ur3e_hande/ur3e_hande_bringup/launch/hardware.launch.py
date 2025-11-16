#!/usr/bin/env python3
# Copyright (c) 2021 PickNik, Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the {copyright_holder} nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

#
# Author: Denis Stogl
# Modified by Clinton Enwerem to provide a unified hardware bringup for the UR3e + Hand-E gripper + RealSense + Perception + MoveIt + PnP.

# Components launched:
#   - RealSense RGB-D camera
#   - UR3e ROS2 driver (hardware.launch.py)
#   - Hand-E gripper joint publisher (state feedback)
#   - Hand-E gripper action server (command execution)
#   - Perception pipeline (voxel --> segmentation --> object metadata)
#   - Object pose action server
#   - MoveIt2 (hardware controllers)
#   - Pick-and-place node (perception or manual position)
# 
# #########################################################

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory as pkg

def generate_launch_description():

    # Core args
    robot_ip = LaunchConfiguration("robot_ip")
    ur_type = LaunchConfiguration("ur_type")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_moveit = LaunchConfiguration("use_moveit")
    use_perception = LaunchConfiguration("use_perception")
    pnp = LaunchConfiguration("pnp")
    launch_rviz = LaunchConfiguration("launch_rviz")

    # Description
    description_launchfile = LaunchConfiguration("description_launchfile")

    # MoveIt parameters
    max_vel_scale = LaunchConfiguration("max_vel_scale")
    max_acc_scale = LaunchConfiguration("max_acc_scale")
    goal_pos_tol = LaunchConfiguration("goal_pos_tol")
    goal_ori_tol = LaunchConfiguration("goal_ori_tol")

    # Realsense + perception
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([
                pkg('realsense2_camera'),
                'launch',
                'rs_launch.py'
            ])]
        ),
        launch_arguments={
            "depth_module.depth_profile": "640x480x30",
            "rgb_camera.color_profile": "640x480x30",
            "pointcloud.enable": "true",
            "align_depth.enable": "true",
            "pointcloud.ordered_pc": "true",
            "enable_infra": "true",
            "enable_infra1": "true",
            "enable_infra2": "true",
            "depth_module.infra_profile": "640x480x30"
        }.items()
    )

    # UR Driver
    ur_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(PathJoinSubstitution([
                pkg("ur_robot_driver"), "launch", "ur_control.launch.py"
            ]))
        ),
        launch_arguments={
            "ur_type": ur_type,
            "robot_ip": robot_ip,
            "launch_rviz": "false",  # we handle RViz below
            "use_sim_time": use_sim_time,
        }.items(),
    )

    # dashboard client node already loaded by ur_control.launch.py
    gripper_action_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "gripper_action_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen"
        )

    start_gripper_after_ur_control = RegisterEventHandler(
    OnProcessStart(
        target_action=ur_control,
        on_start=[gripper_action_spawner],
    ))

    # URDF and RSP
    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(description_launchfile),
        launch_arguments={
            "ur_type": ur_type,
            "robot_ip": robot_ip,
        }.items(),
    )

    # Perception pipeline
    perception_nodes = [
        Node(
            package="ur3e_hande_perception",
            executable="pc_voxel_filter_node",
            name="pc_voxel_filter_node",
            parameters=[{"use_sim_time": use_sim_time}],
            output="screen",
        ),
        Node(
            package="ur3e_hande_perception",
            executable="pc_segmentation_node",
            name="pc_segmentation_node",
            parameters=[{"use_sim_time": use_sim_time}],
            output="screen",
        ),
        Node(
            package="ur3e_hande_perception",
            executable="obj_pose_action_server",
            name="obj_pose_action_server",
            output="screen",
        ),
    ]

    # MoveIt hardware launch
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(PathJoinSubstitution([
                pkg("ur3e_hande_moveit_config"), "launch", "ur3e_hande_moveit.launch.py"
            ]))
        ),
        condition=IfCondition(use_moveit),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "launch_rviz": launch_rviz,
        }.items(),
    )

    # PnP demo
    pnp_demo = Node(
        package="ur3e_hande_moveit_py",
        executable="pnp_demo",
        name="pnp_demo",
        condition=IfCondition(pnp),
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"max_vel_scale": max_vel_scale},
            {"max_acc_scale": max_acc_scale},
            {"goal_pos_tol": goal_pos_tol},
            {"goal_ori_tol": goal_ori_tol},
        ],
    )

    return LaunchDescription([

        # Args
        DeclareLaunchArgument("robot_ip", default_value="192.168.77.22"),
        DeclareLaunchArgument("ur_type", default_value="ur3e"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("use_moveit", default_value="true"),
        DeclareLaunchArgument("use_perception", default_value="false"),
        DeclareLaunchArgument("launch_rviz", default_value="true"),
        DeclareLaunchArgument("description_launchfile",
            default_value=PathJoinSubstitution([
                pkg("ur3e_hande_description"), "launch", "ur3e_hande_rsp.launch.py"
            ])
        ),

        # MoveIt scaling
        DeclareLaunchArgument("max_vel_scale", default_value="0.25"),
        DeclareLaunchArgument("max_acc_scale", default_value="0.25"),
        DeclareLaunchArgument("goal_pos_tol", default_value="0.003"),
        DeclareLaunchArgument("goal_ori_tol", default_value="0.01"),

        # PnP demo
        DeclareLaunchArgument("pnp", default_value="false"),

        # Include subsystems
        description,
        ur_control,
        realsense_launch,
        moveit,
        pnp_demo,
        start_gripper_after_ur_control,

        # launch perception if requested
        *[TimerAction(period=3.0, actions=[node], condition=IfCondition(use_perception))
          for node in perception_nodes]
    ])
