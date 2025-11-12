#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    ur3e_hande_gz_dir = FindPackageShare('ur3e_hande_gz')
    ur3e_hande_moveit_config_dir = FindPackageShare('ur3e_hande_moveit_config_sim')
    moveit_py_dir = FindPackageShare('ur3e_hande_moveit_py')

    view_gz_launch = ExecuteProcess(
        cmd=[
            'ros2', 'launch',
            PathJoinSubstitution([ur3e_hande_gz_dir, 'launch', 'view_gz.launch.py']),
            'launch_rviz:=false', "world_to_spawn:=basic"  
        ],
        output='screen'
    )

    demo_launch = ExecuteProcess(
        cmd=[
            'ros2', 'launch',
            PathJoinSubstitution([moveit_py_dir, 'launch', 'moveit_planning.launch.py']),
        ],
        output='screen'
    )

    return LaunchDescription([
        view_gz_launch,
        demo_launch
    ])
