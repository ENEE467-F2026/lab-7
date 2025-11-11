#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ur3e_bringup_pkg = get_package_share_directory('ur3e_bringup')

    return LaunchDescription([
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         os.path.join(ur3e_bringup_pkg, 'launch', 'hardware.launch.py')
        #     )
        # ),

        # Start realsense driver
        TimerAction(
            period=2.0,
            actions=[
        ExecuteProcess(
            cmd=[
                'ros2', 'launch', 'realsense2_camera', 'rs_launch.py', 'depth_module.depth_profile:=640x480x30',
                'rgb_camera.color_profile:=640x480x30',
                'pointcloud.enable:=true',
                'align_depth.enable:=true',
                'pointcloud.ordered_pc:=true',
                'enable_infra:=true',
                'enable_infra1:=true',
                'enable_infra2:=true',
                'depth_module.infra_profile:=640x480x30'
            ],
            output='screen'
              ),
            ]),

        # Start ros2_control
        TimerAction(
            period=5.0,
            actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'control', 'set_controller_state',
                    'scaled_joint_trajectory_controller', 'active',
                    '-c', '/controller_manager'
                ],
                output='screen'
            )]),

        # Swap controllers
        TimerAction(
            period=7.0,
            actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'control', 'switch_controllers', '-c',
                    '/controller_manager', '--deactivate',
                    'scaled_joint_trajectory_controller', '--activate', 'forward_velocity_controller'
                ],
                output='screen'
            )]),


        # Gripper action server
        TimerAction(
            period=9.0,
            actions=[
                Node(
                    package='robotiq_hande_ros2_driver',
                    executable='gripper_action_server',  
                    name='gripper_action_server',
                    output='screen',
                    parameters=[
                        {'robot_ip': '192.168.77.22'},
                        {'p_diff_thresh': 5},
                        {'u_diff_thresh': 4},
                        {'f_diff_thresh': 5}
                    ],
                )]),

        # Gripper JSP
        TimerAction(
            period=10.0,
            actions=[
            Node(
                package='robotiq_hande_ros2_driver',
                executable='gripper_joint_publisher',
                name='gripper_joint_publisher',
                output='screen'
        )]), 

        TimerAction(
            period=12.0,
            actions=[
                Node(
                     package='robotiq_hande_ros2_driver',
                      executable='simple_teleop_hande',
                      name='simple_teleop_hande',
                      output='screen'
                )
    ]),

        # data logger node
        TimerAction(
            period=14.0,
            actions=[
                Node(
                     package='robotiq_hande_ros2_driver',
                      executable='data_logger',
                      name='data_logger',
                      output='screen'
                )
    ])


    ])
