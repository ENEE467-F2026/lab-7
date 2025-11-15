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
# Modified by: Clinton Enwerem

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from ur3e_moveit_config.launch_common import load_yaml


def launch_setup(context, *args, **kwargs):
    # Launch configurations
    use_sim_time = LaunchConfiguration("use_sim_time")
    prefix = LaunchConfiguration("prefix")
    namespace = LaunchConfiguration("namespace")
    launch_rviz = LaunchConfiguration("launch_rviz")
    warehouse_sqlite_path = LaunchConfiguration("warehouse_sqlite_path")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    kinematics_params_file = LaunchConfiguration("kinematics_params_file")

    # Resolve package paths (static resolution)
    desc_pkg = FindPackageShare("ur3e_hande_description").find("ur3e_hande_description")
    moveit_pkg = FindPackageShare("ur3e_hande_moveit_config").find("ur3e_hande_moveit_config")

    urdf_path = f"{desc_pkg}/urdf/ur3e_hande_hw.urdf.xacro"
    srdf_path = f"{moveit_pkg}/config/ur3e_hande.srdf"
    kin_path = f"{moveit_pkg}/config/kinematics.yaml"
    ompl_path = f"{moveit_pkg}/config/ompl_planning.yaml"
    ctrl_path = f"{moveit_pkg}/config/moveit_controllers.yaml"
    ros2_ctrl_path = f"{moveit_pkg}/config/ros2_controllers.yaml"
    rviz_config_file = f"{moveit_pkg}/config/moveit_pnp.rviz"
    kinematics_params = PathJoinSubstitution(
        [FindPackageShare(os.path.join(desc_pkg), 
                          "config", 
                          kinematics_params_file)]
    )
    

    # Build MoveIt config from scratch
    moveit_config = (
        MoveItConfigsBuilder("ur3e_hande", package_name="ur3e_hande_moveit_config")
        .robot_description(file_path=urdf_path)
        .robot_description_semantic(file_path=srdf_path)
        .robot_description_kinematics(file_path=kin_path)
        .planning_pipelines(default_planning_pipeline="ompl", pipelines=["ompl"])
        .trajectory_execution(file_path=ctrl_path)
        .to_moveit_configs()
    )

    # Load additional YAMLs
    moveit_controllers = load_yaml("ur3e_hande_moveit_config", "config/moveit_controllers.yaml")
    controllers_yaml = load_yaml("ur3e_hande_moveit_config", "config/ros2_controllers.yaml")
    ompl_planning_yaml = load_yaml("ur3e_hande_moveit_config", "config/ompl_planning.yaml")
    joint_limits = load_yaml("ur3e_hande_moveit_config", "config/joint_limits.yaml")

    # OMPL planner config
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": (
                "default_planner_request_adapters/AddTimeOptimalParameterization "
                "default_planner_request_adapters/FixWorkspaceBounds "
                "default_planner_request_adapters/FixStartStateBounds "
                "default_planner_request_adapters/FixStartStateCollision "
                "default_planner_request_adapters/FixStartStatePathConstraints"
            ),
            "start_state_max_bounds_error": 0.1,
        }
    }
    if ompl_planning_yaml:
        ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    warehouse_ros_config = {
        "warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
        "warehouse_host": warehouse_sqlite_path,
    }

    # Controller spawners (unchanged)
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    ur_joint_traj_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    gripper_action_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_action_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # MoveIt move_group node using the constructed moveit_config
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        namespace=namespace,
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": use_sim_time},
            controllers_yaml,
            {"robot_description_planning": {"joint_limits": joint_limits} if joint_limits else {}},
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            warehouse_ros_config,
        ],
        remappings=[
            ("/joint_states", "/joint_states"),
        ],
    )

    # RViz node using the generated config
    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(launch_rviz),
        executable="rviz2",
        name="rviz2_moveit",
        namespace=namespace,
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": use_sim_time},
        ],
    )

    # Ensure controllers spawn before move_group
    controller_event_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=ur_joint_traj_spawner,
            on_exit=[gripper_action_spawner, move_group_node, rviz_node],
        )
    )

    nodes_to_start = [
        joint_state_broadcaster_spawner,
        ur_joint_traj_spawner,
        controller_event_handler,
    ]
    return nodes_to_start


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("use_fake_hardware", default_value="false"),
        DeclareLaunchArgument("namespace", default_value=""),
        DeclareLaunchArgument("prefix", default_value='""'),
        DeclareLaunchArgument("launch_rviz", default_value="true"),
        DeclareLaunchArgument("launch_servo", default_value="false"),
        DeclareLaunchArgument("moveit_config_package", default_value="ur3e_hande_moveit_config"),
        DeclareLaunchArgument("ur_hande_description_package", default_value="ur3e_hande_description"),
        DeclareLaunchArgument("ur_hande_description_file", default_value="ur3e_hande_hw.urdf.xacro"),
        DeclareLaunchArgument("warehouse_sqlite_path", default_value=os.path.expanduser("~/.ros/warehouse_ros.sqlite")),
        DeclareLaunchArgument("kinematics_params_file", default_value=os.environ.get("KINEMATICS_CONFIG_FILE", "/home/robot/kinematic_config/ur3e_mrc.yaml")),
    ]
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])