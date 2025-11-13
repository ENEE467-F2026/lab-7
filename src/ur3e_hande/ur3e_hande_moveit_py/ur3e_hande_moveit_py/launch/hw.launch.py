#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ur3e_bringup_pkg = get_package_share_directory('ur3e_bringup')
    ur3e_hande_moveit_pkg = get_package_share_directory('ur3e_hande_moveit_config')
    use_perception = LaunchConfiguration("use_perception")
    obj_pos = LaunchConfiguration("obj_pos")
    robot_ip = LaunchConfiguration("robot_ip")
    launch_rviz = LaunchConfiguration("launch_rviz")

    declared_arguments = [
        DeclareLaunchArgument("use_perception", default_value="true", description="Whether to use the perception module to detect object pose. \n Must provide the position of the object to be picked as a launch argument (obj_pos) if false, unless the launch file will throw an error."),
        DeclareLaunchArgument("obj_pos", default_value="0.33931674 0.3942382 0.2380788", description="The position (specified as space-separated floats x y z) of the object to pick.\n Only used if use_perception is set to false."),
        DeclareLaunchArgument("robot_ip", default_value="192.168.77.22", description="IP address of the robot."),
        DeclareLaunchArgument("launch_rviz", default_value="false", description="Whether to launch RViz."),
    ]

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
)
    
    return LaunchDescription(declared_arguments + [
        TimerAction(
            period=0.0,
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
                output='log'
              ),
            ]),

        # start hardware launch
        TimerAction(
            period=3.0,
            actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                os.path.join(ur3e_bringup_pkg, 'launch', 'hardware.launch.py')
                ),
                launch_arguments={
                'robot_ip': robot_ip,
                'launch_rviz': launch_rviz
                }.items()
            )
            ]
        ),

        # Run perception nodes
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
                {"use_sim_time": True},
                {"input_topic": "/camera/camera/depth/color/points"},
                {"output_topic": "/filtered_cloud"},
                {"leaf_size": 0.005},
                {"crop_enabled": True},
                # {"crop_bounds": [0.33, 2.00, -0.72, 1.00, -0.50, 0.45]},
                ],
            )
            ]
        ),

        TimerAction(
            period=6.0,
            condition=IfCondition(use_perception),
            actions=[
            Node(
                package="ur3e_hande_perception",
                executable="pc_segmentation_node",
                name="pc_segmentation_node",
                output="screen",
                parameters=[
                {"use_sim_time": True},
                {"input_topic": "/filtered_cloud"}
                ],
            )
            ]
        ),

        # Gripper JSP
        TimerAction(
            period=8.0,
            actions=[
            Node(
                package='robotiq_hande_ros2_driver',
                executable='gripper_joint_publisher',
                name='gripper_joint_publisher',
                output='screen'
        )]), 

        # launch ur3e_hande_moveit.launch
        TimerAction(
            period=10.0,
            actions=[
            ExecuteProcess(
                cmd=[
                'ros2', 'launch', 'ur3e_hande_moveit_config', 'ur3e_hande_moveit.launch.py'
                ],
                output='screen'
            )
            ]
        ),

        # gripper action server (serves gripper JointState() object)
        TimerAction(
            period=12.0,
            actions=[
                Node(
                    package='robotiq_hande_ros2_driver',
                    executable='gripper_action_server',  
                    name='gripper_action_server',
                    parameters=[
                        {'robot_ip': LaunchConfiguration("robot_ip")},
                        {'p_diff_thresh': 5},
                        {'u_diff_thresh': 4},
                        {'f_diff_thresh': 5}
                    ],
                    output='screen'
                )]),

        # pick-and-place node (with perception)
        TimerAction(
            period=14.0,
            condition=IfCondition(use_perception),
            actions=[
                Node(
                     package='ur3e_hande_moveit_py',
                      executable='pnp_demo.py',
                      name='pnp_demo',
                      output='screen'
                )
            ]),

        # pick-and-place node (manual object position specification)
        TimerAction(
            period=14.0,
            condition=UnlessCondition(use_perception),
            actions=[
                Node(
                     package='ur3e_hande_moveit_py',
                     executable='pnp_demo',
                     name='pnp_demo',
                     output='screen',
                     parameters=[
                        {'obj_pos': obj_pos},
                     ]
                )
            ])
])
