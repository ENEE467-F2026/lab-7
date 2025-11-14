#!/usr/bin/env python3
"""
Launch the full Lab 7 autonomous manipulation stack in simulation.

Brings up:
  • Gazebo + MoveIt 2 for UR3e-HandE
  • Point cloud voxel filter + segmentation nodes
  • Object Pose Server (action server for perception --> planning)
  • Pick-and-Place client (MoveIt2 + gripper control)

Usage:
  ros2 launch ur3e_hande_moveit_py sim.launch.py
  ros2 launch ur3e_hande_moveit_py sim.launch.py \
      pregrasp_z_offset:=0.04 place_offset_y:=0.05 velocity_scale:=0.3
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


def generate_launch_description():
    # Launch configurations
    launch_rviz = LaunchConfiguration("launch_rviz")
    velocity_scale = LaunchConfiguration("velocity_scale")
    pregrasp_z_offset = LaunchConfiguration("pregrasp_z_offset")
    place_offset_y = LaunchConfiguration("place_offset_y")
    world_to_spawn = LaunchConfiguration("world_to_spawn")
    pnp = LaunchConfiguration("pnp")
    print_metrics = LaunchConfiguration("print_metrics")
    use_perception = LaunchConfiguration("use_perception")

    # Declared arguments
    declared_arguments = [
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Launch RViz with the Gazebo + MoveIt scene.",
        ),
        DeclareLaunchArgument(
            "velocity_scale",
            default_value="0.2",
            description="Velocity/acceleration scaling for MoveIt2 trajectories.",
        ),
        DeclareLaunchArgument(
            "pregrasp_z_offset",
            default_value="0.0",
            description="Extra Z-offset added to the pre-grasp pose (meters).",
        ),
        DeclareLaunchArgument(
            "place_offset_y",
            default_value="0.0",
            description="Lateral offset along Y for placement (meters).",
        ),
        DeclareLaunchArgument(
            "world_to_spawn",
            default_value="smallbox",
            description="Gazebo world to spawn (basic, bookshelf, bin, aruco, or smallbox).",
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
            "use_perception",
            default_value="true",
            description="Whether to use perception nodes for object pose estimation.",
        ),
    ]

    # Gazebo + MoveIt2 simulation
    gz_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ur3e_hande_moveit_py"), "launch", "gz_moveit.launch.py"]
            )
        ),
        launch_arguments={
            "launch_rviz": launch_rviz,
            "world_to_spawn": world_to_spawn,
        }.items(),
    )

    # Perception nodes
    pc_voxel_filter_node = Node(
        package="ur3e_hande_perception",
        executable="pc_voxel_filter_node",
        name="pc_voxel_filter_node",
        output="screen",
        condition=IfCondition(use_perception),
        parameters=[
            {"use_sim_time": True},
            {"input_topic": "/rgbd_camera/points"},
            {"output_topic": "/filtered_cloud"},
            {"leaf_size": 0.005},
            {"crop_enabled": True},
            {"stop_after_first": True},
            # {"crop_bounds": [0.33, 2.00, -0.72, 1.00, -0.50, 0.45]},
        ],
    )

    pc_segmentation_node = Node(
        package="ur3e_hande_perception",
        executable="pc_segmentation_node",
        name="pc_segmentation_node",
        output="screen",
        condition=IfCondition(use_perception),
        parameters=[
            {"use_sim_time": True},
            {"input_topic": "/filtered_cloud"},
            {"stop_after_first_pub": True},
        ],
    )

    # Object Pose Action Server
    obj_pose_action_server = TimerAction(
        period=4.0,  # start a few seconds after perception
        actions=[
            Node(
                package="ur3e_hande_perception",
                executable="obj_pose_action_server",
                name="object_pose_server",
                output="screen",
                condition=IfCondition(use_perception),
                parameters=[
                    {"use_sim_time": True},
                ],
            )
        ],
    )

    # Pick-and-Place Client
    pick_and_place = TimerAction(
        period=10.0,  # wait for perception + pose server to initialize
        actions=[
            Node(
                package="ur3e_hande_moveit_py",
                executable="pnp_demo_sim",
                name="pnp_demo_sim",
                output="screen",
                condition=IfCondition(pnp),
                parameters=[
                    {"use_sim_time": True},
                    {"velocity_scale": ParameterValue(velocity_scale, value_type=float)},
                    {"pregrasp_z_offset": ParameterValue(pregrasp_z_offset, value_type=float)},
                    {"place_offset_y": ParameterValue(place_offset_y, value_type=float)},
                    # If omitted, PickAndPlaceSim will query GetTargetObjPose
                ],
            )
        ],
    )

    # Metrics Logger Node
    metrics_logger_node = TimerAction(
        period=10.0,  # wait for other nodes to initialize
        actions=[
            Node(
                package="ur3e_hande_moveit_py",
                executable="metrics_logger",
                name="pnp_metrics_logger",
                output="screen",
                condition=IfCondition(
                PythonExpression([
                    "'",
                    pnp,
                    "' == 'true' and '",
                    print_metrics,
                    "' == 'true' and '",
                    use_perception,
                    "' == 'true'"
                ])
                ),
                parameters=[
                    {"use_sim_time": True},
                    {"csv_path": "pnp_metrics_sim.csv"},
                    {"base_frame": "base_link"},
                    {"ee_frame": "tool0"},
                ],
            )
        ],
    )

    # Final launch sequence
    return LaunchDescription(
        declared_arguments
        + [
            gz_moveit_launch,
            pc_voxel_filter_node,
            pc_segmentation_node,
            obj_pose_action_server,
            pick_and_place,
            metrics_logger_node,
        ]
    )
