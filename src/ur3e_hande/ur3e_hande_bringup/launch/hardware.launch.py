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

import os
from os import path
from os.path import expanduser

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterFile

from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
    AndSubstitution,
    NotSubstitution,
)
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def launch_setup(context):
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
    controller_spawner_timeout = LaunchConfiguration("controller_spawner_timeout")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")
    activate_joint_controller = LaunchConfiguration("activate_joint_controller")
    headless_mode = LaunchConfiguration("headless_mode")
    launch_dashboard_client = LaunchConfiguration("launch_dashboard_client")
    tool_device_name = LaunchConfiguration("tool_device_name")
    tool_tcp_port = LaunchConfiguration("tool_tcp_port")


    # RealSense Bringup
    realsense_node = TimerAction(
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

    # Control Node
    control_node = TimerAction(
            period=1.0,
            actions=[
               Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[
                    LaunchConfiguration("update_rate_config_file"),
                    ParameterFile(controllers_file, allow_substs=True),
                    # We use the tf_prefix as substitution in there, so that's why we keep it as an
                    # argument for this launchfile
                ],
                remappings=[
                    ("~/robot_description", "/robot_description"),
                ],
                output="screen",
            )
            ],
    )

    # UR3e Hardware Bringup
    dashboard_client_node= TimerAction(
            period=3.0,
            actions=[
                Node(
                package="ur_robot_driver",
                condition=IfCondition(
                    AndSubstitution(launch_dashboard_client, NotSubstitution(use_mock_hardware))
                ),
                executable="dashboard_client",
                name="dashboard_client",
                output="screen",
                emulate_tty=True,
                parameters=[{"robot_ip": robot_ip}],
            )])
    
    robot_state_helper_node = TimerAction(
            period=4.0,
            actions=[
            Node(
                package="ur_robot_driver",
                executable="robot_state_helper",
                name="ur_robot_state_helper",
                output="screen",
                condition=UnlessCondition(use_mock_hardware),
                parameters=[
                    {"headless_mode": headless_mode},
                    {"robot_ip": robot_ip},
                ],
            )])

    tool_communication_node = TimerAction(
            period=5.0,
            actions=[Node(
                package="ur_robot_driver",
                condition=IfCondition(use_tool_communication),
                executable="tool_communication.py",
                name="ur_tool_comm",
                output="screen",
                parameters=[
                    {
                        "robot_ip": robot_ip,
                        "tcp_port": tool_tcp_port,
                        "device_name": tool_device_name,
                    }
                ],
            )])

    urscript_interface =  TimerAction(
            period=5.0,
            actions=[Node(
                package="ur_robot_driver",
                executable="urscript_interface",
                parameters=[{"robot_ip": robot_ip}],
                output="screen",
                condition=UnlessCondition(use_mock_hardware),
            )])

    controller_stopper_node =  TimerAction(
            period=6.0,
            actions=[Node(
                package="ur_robot_driver",
                executable="controller_stopper_node",
                name="controller_stopper",
                output="screen",
                emulate_tty=True,
                condition=UnlessCondition(use_mock_hardware),
                parameters=[
                    {"headless_mode": headless_mode},
                    {"joint_controller_active": activate_joint_controller},
                    {
                        "consistent_controllers": [
                        "io_and_status_controller",
                        "force_torque_sensor_broadcaster",
                        "joint_state_broadcaster",
                        "speed_scaling_state_broadcaster",
                        "tcp_pose_broadcaster",
                        "ur_configuration_controller",
                        ]
                    },
                ],
            )])

    # trajectory_until_node = TimerAction(
    #         period=7.0,
    #         actions=[Node(
    #             package="ur_robot_driver",
    #             executable="trajectory_until_node",
    #             name="trajectory_until_node",
    #             output="screen",
    #             parameters=[
    #                 {
    #                 "motion_controller": initial_joint_controller,
    #                 },
    #             ],
    #         )])

    # rviz2
    rviz_node = TimerAction(
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
    
    # Perception Nodes
    pc_voxel_filter_node = TimerAction(
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
                )])
    
    pc_segmentation_node = TimerAction(
            period=9.0,
            condition=IfCondition(use_perception),
            actions=[Node(
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
                )])
    
    obj_pose_action_server =  TimerAction(
            period=9.0,
            condition=IfCondition(use_perception),
            actions=[Node(
                    package="ur3e_hande_perception",
                    executable="obj_pose_action_server",
                    name="object_pose_server",
                    output="screen",
                )])

    # MoveIt2 for Hardware
    movieit_launch = TimerAction(
            period=12.0,
            condition=IfCondition(use_moveit),
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(get_package_share_directory("ur3e_hande_moveit_config"), "launch", "ur3e_hande_moveit.launch.py")
                    ),
                    launch_arguments={
                        "use_sim_time": use_sim_time,
                        "log_level": log_level,
                        "launch_rviz": launch_rviz,
                    }.items(),
                )
            ],
        )

    # Pick-and-Place Node
    pnp_node = TimerAction(
            period=12.0,
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

    # Spawn controllers
    def controller_spawner(controllers, active=True):
        inactive_flags = ["--inactive"] if not active else []
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "--controller-manager",
                "/controller_manager",
                "--controller-manager-timeout",
                controller_spawner_timeout,
            ]
            + inactive_flags
            + controllers,
        )

    controllers_active = [
        "joint_state_broadcaster",
        "speed_scaling_state_broadcaster",
        "force_torque_sensor_broadcaster",
        "tcp_pose_broadcaster",
        "ur_configuration_controller",
    ]
    controllers_inactive = [
        "scaled_joint_trajectory_controller",
        "joint_trajectory_controller",
        "forward_velocity_controller",
        "forward_position_controller",
        "forward_effort_controller",
        "force_mode_controller",
        "passthrough_trajectory_controller",
        "freedrive_mode_controller",
        "tool_contact_controller",
    ]
    if activate_joint_controller.perform(context) == "true":
        controllers_active.append(initial_joint_controller.perform(context))
        controllers_inactive.remove(initial_joint_controller.perform(context))

    if use_mock_hardware.perform(context) == "true":
        controllers_active.remove("tcp_pose_broadcaster")

    controller_spawners = [
        controller_spawner(controllers_active),
        controller_spawner(controllers_inactive, active=False),
    ]

    rsp = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(description_launchfile),
        launch_arguments={
            "robot_ip": robot_ip,
            "ur_type": ur_type,
        }.items(),
    )

    nodes_to_start = [
        realsense_node,
        control_node,
        dashboard_client_node,
        robot_state_helper_node,
        tool_communication_node,
        controller_stopper_node,
        urscript_interface,
        rsp,
        rviz_node,
        # trajectory_until_node,
        pc_voxel_filter_node,
        pc_segmentation_node,
        obj_pose_action_server, 
        movieit_launch,
        pnp_node

    ] + controller_spawners

    return nodes_to_start

def generate_launch_description():


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
            name="update_rate_config_file",
            default_value=[
                PathJoinSubstitution(
                    [
                        FindPackageShare("ur3e_hande_description"),
                        "config",
                    ]
                ),
                "ur3e_update_rate.yaml",
            ],
        ),
         DeclareLaunchArgument(
            "controller_spawner_timeout",
            default_value="10",
            description="Timeout used when spawning controllers.",
        ),
        DeclareLaunchArgument(
            "mock_sensor_commands",
            default_value="false",
            description="Enable mock command interfaces for sensors used for simple simulations. "
            "Used only if 'use_mock_hardware' parameter is true.",
        ),
        DeclareLaunchArgument(
            "headless_mode",
            default_value="false",
            description="Enable headless mode for robot control",
        ),

        DeclareLaunchArgument(
            "activate_joint_controller",
            default_value="true",
            description="Activate loaded joint controller.",
        ),
        DeclareLaunchArgument(
            "tool_stop_bits",
            default_value="1",
            description="Stop bits configuration for serial communication. Only effective, if "
            "use_tool_communication is set to True.",
        ),
        DeclareLaunchArgument(
            "initial_joint_controller",
            default_value="scaled_joint_trajectory_controller",
            choices=[
                "scaled_joint_trajectory_controller",
                "joint_trajectory_controller",
                "forward_velocity_controller",
                "forward_position_controller",
                "freedrive_mode_controller",
                "passthrough_trajectory_controller",
            ],
            description="Initially loaded robot controller.",
        ),
        DeclareLaunchArgument(
            "launch_dashboard_client",
            default_value="true",
            description="Launch Dashboard Client?",
        ),
        DeclareLaunchArgument(
            "tool_baud_rate",
            default_value="115200",
            description="Baud rate configuration for serial communication. Only effective, if "
            "use_tool_communication is set to True.",
        ),
        DeclareLaunchArgument(
            "tool_parity",
            default_value="0",
            description="Parity configuration for serial communication. Only effective, if "
            "use_tool_communication is set to True.",
        ),
        DeclareLaunchArgument(
            "tool_tx_idle_chars",
            default_value="3.5",
            description="TX idle chars configuration for serial communication. Only effective, "
            "if use_tool_communication is set to True.",
        ),
        DeclareLaunchArgument(
            "tool_device_name",
            default_value="/tmp/ttyUR",
            description="File descriptor that will be generated for the tool communication device. "
            "The user has be be allowed to write to this location. "
            "Only effective, if use_tool_communication is set to True.",
        ),
        DeclareLaunchArgument(
            "controllers_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("ur3e_hande_description"), "config", "controllers.yaml"]
            ),
            description="YAML file with the combined controllers configuration.",
        ),
        DeclareLaunchArgument(
            "reverse_ip",
            default_value="0.0.0.0",
            description="IP that will be used for the robot controller to communicate back to the driver.",
        ),
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="false",
            description="Start robot with mock hardware mirroring command to its states.",
        ),
        DeclareLaunchArgument(
            "tool_tcp_port",
            default_value="54321",
            description="Remote port that will be used for bridging the tool's serial device. "
            "Only effective, if use_tool_communication is set to True.",
        ),
        DeclareLaunchArgument(
            "tool_voltage",
            default_value="0",  # 0 being a conservative value that won't destroy anything
            description="Tool voltage that will be setup.",
        ),
        DeclareLaunchArgument(
            "script_command_port",
            default_value="50004",
            description="Port that will be opened to forward URScript commands to the robot.",
        ),
        DeclareLaunchArgument(
            "trajectory_port",
            default_value="50003",
            description="Port that will be opened for trajectory control.",
        ),
        DeclareLaunchArgument(
            "reverse_port",
            default_value="50001",
            description="Port that will be opened to send cyclic instructions from the driver to the robot controller.",
        ),
        DeclareLaunchArgument(
            "script_sender_port",
            default_value="50002",
            description="The driver will offer an interface to query the external_control URScript on this port.",
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
    

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
