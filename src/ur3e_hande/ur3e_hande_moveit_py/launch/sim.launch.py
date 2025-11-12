#!/usr/bin/env python3
"""
Launch the full Lab 7 autonomous manipulation stack in simulation.

Brings up:
  • Gazebo Harmonic simulation with MoveIt 2 (UR3e-HandE)
  • YOLO + point cloud perception node
  • PyMoveIt2 pick-and-place node using ros2_control gripper action

ros2 launch ur3e_hande_lab7_bringup lab7_autonomous_sim.launch.py

ros2 launch ur3e_hande_lab7_bringup lab7_autonomous_sim.launch.py \
    pregrasp_z_offset:=0.04 place_offset_y:=0.05 velocity_scale:=0.3

"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch Configurations
    launch_rviz = LaunchConfiguration("launch_rviz")
    velocity_scale = LaunchConfiguration("velocity_scale")
    pregrasp_z_offset = LaunchConfiguration("pregrasp_z_offset")
    place_offset_y = LaunchConfiguration("place_offset_y")
    pose_avg_window = LaunchConfiguration("pose_avg_window")
    world_to_spawn = LaunchConfiguration("world_to_spawn")

    # Declared Launch Arguments
    declared_arguments = [
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Launch RViz with the Gazebo + MoveIt simulation scene.",
        ),
        DeclareLaunchArgument(
            "velocity_scale",
            default_value="0.2",
            description="Velocity/acceleration scaling factor for MoveIt execution.",
        ),
        DeclareLaunchArgument(
            "pregrasp_z_offset",
            default_value="0.0",
            description="Extra vertical offset (m) added to the pre-grasp waypoint.",
        ),
        DeclareLaunchArgument(
            "place_offset_y",
            default_value="0.0",
            description="Lateral offset (m) applied to the placement target along Y-axis.",
        ),
        DeclareLaunchArgument(
            "pose_avg_window",
            default_value="1",
            description="Number of perception pose estimates to average before publishing.",
        ),
        DeclareLaunchArgument(
            "world_to_spawn",
            default_value="smallbox",
            description="Gazebo world to spawn (basic, bookshelf, bin, aruco, or smallbox).",
        ),
    ]

    # Gazebo Simulation
    gz_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ur3e_hande_gz"), "launch", "gz_moveit.launch.py"]
            )
        ),
        launch_arguments={
            "launch_rviz": launch_rviz,
            "world_to_spawn": world_to_spawn,
        }.items(),
    )

    # Perception node
    perception_node = Node(
        package="ur3e_hande_perception",
        executable="yolo_pc_pose_estimation.py",
        name="yolo_pc_pose_estimator",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"rs_color_topic": "/rgbd_camera/image"},
            {"rs_depth_topic": "/rgbd_camera/depth_image"},
            {"rs_color_info_topic": "/rgbd_camera/camera_info"},
            {"rs_pc_topic": "/rgbd_camera/points"},
            {"pose_avg_window": ParameterValue(pose_avg_window, value_type=int)},
        ],
    )

    # Pick-and-Place Node (MoveIt2 + ros2_control gripper)
    pick_and_place = Node(
        package="ur3e_hande_moveit_py",
        executable="pnp_demo_sim.py",
        name="lab7_pick_and_place",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"velocity_scale": ParameterValue(velocity_scale, value_type=float)},
            {"pregrasp_z_offset": ParameterValue(pregrasp_z_offset, value_type=float)},
            {"place_offset_y": ParameterValue(place_offset_y, value_type=float)},
        ],
    )

    # Final LaunchDescription
    return LaunchDescription(
        declared_arguments
        + [
            gz_moveit_launch,
            perception_node,
            pick_and_place,
        ]
    )
