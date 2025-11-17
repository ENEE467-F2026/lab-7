#!/usr/bin/env python3

"""
Launch file: Gripper and Joint State Integration

This launch:
  - Publishes Robotiq joint states (1-DOF parallel jaw)
  - Merges UR3e joint states + Hand-E joint state into a unified /joint_states
  - Starts the Robotiq gripper action server for MoveIt or custom clients

Author: Clinton Enwerem
Developed for the course ENEE467: Robotics Projects Laboratory, Fall 2025, University of Maryland, College Park, MD.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # Publishes /robotiq_joint_states 
    gripper_joint_pub = Node(
        package="robotiq_api_wrapper",
        executable="gripper_joint_publisher",
        name="gripper_joint_publisher",
        output="screen",
    )

    # Merges UR joint_states and Robotiq joint_states into /joint_states
    joint_state_merger = Node(
        package="ur3e_hande_moveit_config",
        executable="joint_state_merger",
        name="joint_state_merger",
        output="screen",
        parameters=[
            {"merge_topic": True},         
            {"target_topic": "/joint_states"},
        ]
    )

    # Action server that exposes gripper control:
    #   action: gripper_action/GripperAction
    gripper_action_server = Node(
        package="robotiq_api_wrapper",
        executable="gripper_action_server",
        name="gripper_action_server",
        output="screen",
    )

    return LaunchDescription([
        gripper_joint_pub,
        joint_state_merger,
        gripper_action_server,
    ])
