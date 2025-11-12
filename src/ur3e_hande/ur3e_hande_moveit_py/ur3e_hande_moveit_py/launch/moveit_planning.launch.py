#!/usr/bin/env python3
"""
Launch MoveIt 2 for UR3e + Hand-E robot.
Compatible with ROS 2 Jazzy.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # Launch Arguments
    declared_arguments = [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation clock if true.",
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=PathJoinSubstitution([
                FindPackageShare("ur3e_hande_moveit_config_sim"),
                "config",
                "moveit_pnp.rviz",
            ]),
            description="Path to RViz configuration file.",
        ),
    ]

    use_sim_time = LaunchConfiguration("use_sim_time")
    rviz_config = LaunchConfiguration("rviz_config")

    # Resolve package paths
    desc_pkg = FindPackageShare("ur3e_hande_description").find("ur3e_hande_description")
    moveit_pkg = FindPackageShare("ur3e_hande_moveit_config_sim").find("ur3e_hande_moveit_config_sim")

    urdf_path = f"{desc_pkg}/urdf/ur3e_hande.urdf.xacro"
    srdf_path = f"{moveit_pkg}/config/ur3e_hande.srdf"
    kin_path = f"{moveit_pkg}/config/kinematics.yaml"
    ompl_path = f"{moveit_pkg}/config/ompl_planning.yaml"
    ctrl_path = f"{moveit_pkg}/config/moveit_controllers.yaml"

    # Build MoveIt Config explicitly
    moveit_config = (
        MoveItConfigsBuilder("ur3e_hande", package_name="ur3e_hande_moveit_config_sim")
        .robot_description(file_path=urdf_path)
        .robot_description_semantic(file_path=srdf_path)
        .robot_description_kinematics(file_path=kin_path)
        .planning_pipelines(default_planning_pipeline="ompl", pipelines=["ompl"])
        .trajectory_execution(file_path=ctrl_path)
        .to_moveit_configs()
    )

    # MoveGroup Node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": use_sim_time},
        ],
    )

    # RViz Node 
    rviz_node = TimerAction(
        period=4.0,
        actions=[
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", rviz_config],
                parameters=[
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    {"use_sim_time": use_sim_time},
                ],
                output="screen",
            )
        ],
    )

    return LaunchDescription(declared_arguments + [move_group_node, rviz_node])
