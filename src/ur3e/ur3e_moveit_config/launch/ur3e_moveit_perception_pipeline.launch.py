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

import os, yaml
import numpy as np

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ur3e_moveit_config.launch_common import load_yaml
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

from geometry_msgs.msg import TransformStamped
from transforms3d.euler import euler2quat

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)

# # Get params config 
# Get params
ROOT_PKG_NAME = 'ur3e_moveit_config'
PARAMS_FILE_NAME = 'realsense_params.yaml'
PARMS_DIR = os.path.join(get_package_share_directory(ROOT_PKG_NAME), "config", PARAMS_FILE_NAME)

#  Get params
with open(PARMS_DIR, 'r') as stream:
    params = yaml.safe_load(stream)

# Compute variable params not set in params_file
ur_base_xyz = [0.546, 0.7315125, params["extrinsic_params"]["vention_table_height"]]
ur_base_rpy = [0, 0, -1.5707963]
alp_xyz = [1.0451687500000002, 0.78628125, params["extrinsic_params"]["vention_table_height"]]
alp_rpy = [0, 0, 0.5314427836344855]
alp_to_cam_xyz = [0, params["extrinsic_params"]["camera_mount_bar_length"], params["extrinsic_params"]["camera_mount_height"]]

WORLD_FRAME = params["world_frame_id"]

# Intrinsics
RGBD_WIDTH = params["intrinsic_params"]["rgbd"]["resolution"]["width"]
RGBD_HEIGHT = params["intrinsic_params"]["rgbd"]["resolution"]["height"]
RGBD_WINDOW_SIZE = np.array(params["intrinsic_params"]["rgbd"]["window_size"])

DEPTH_WIDTH = params["intrinsic_params"]["depth"]["resolution"]["width"]
DEPTH_HEIGHT = params["intrinsic_params"]["depth"]["resolution"]["height"]
DEPTH_WINDOW_SIZE = np.array(params["intrinsic_params"]["depth"]["window_size"])

CAMERA_NAMESPACE = params['robot_name']
CAMERA_NAME = params["device_type"]
RGBD_TOPIC = f"/{CAMERA_NAMESPACE}/{CAMERA_NAME}/color/image_raw"
DEPTH_TOPIC = f"/{CAMERA_NAMESPACE}/{CAMERA_NAME}/depth/image_rect_raw"
POINT_CLOUD_TOPIC = f"/{CAMERA_NAMESPACE}/{CAMERA_NAME}/depth/color/points"
ALIGNED_DEPTH2COLOR_TOPIC = f"/{CAMERA_NAMESPACE}/{CAMERA_NAME}/aligned_depth_to_color/image_raw"

DISPLAY_IMAGES = params["display_images"]
QUEUE_SIZE = int(params["queue_size"])

# MoveIt2
MAX_VELOCITY = params['moveit2']['max_velocity']
MAX_ACCN = params['moveit2']['max_accn']

# Camera intrinsic parameters
FX = params["intrinsic_params"]["rgbd"]["focal_length"]["x"]
FY = params["intrinsic_params"]["rgbd"]["focal_length"]["y"]
PPX = params["intrinsic_params"]["rgbd"]["principal_point"]["x"]
PPY = params["intrinsic_params"]["rgbd"]["principal_point"]["y"]

# Target color for segmentation (red box)
LOWER_BGR = (0, 0, 100)
UPPER_BGR = (50, 50, 255)

# Pre-calibration pose
TCP_PRECALIBRATION_POSE = params["tcp_precalibration_pose"]

# TF
d435i_pose = [0.0, 0.6731, 1.27465, 0.0, -0.52, 1.57] #[-0.0, 0.1, 0.36, 0, 0.0, 3.0] #params["d435i_pose"]
    
# Extract translation (x, y, z) and rotation (roll, pitch, yaw)
translation = d435i_pose[:3]
roll, pitch, yaw = d435i_pose[3:]

# Convert Euler angles to quaternion
qw, qx, qy, qz = euler2quat(roll, pitch, yaw)

REALSENSE_PACKAGE_NAME = 'realsense2_camera'
# REALSENSE_LAUNCH_PATH = os.path.join(get_package_share_directory(REALSENSE_PACKAGE_NAME), 'examples', 'pointcloud', 'rs_pointcloud_launch.py')
REALSENSE_LAUNCH_PATH = os.path.join(get_package_share_directory(REALSENSE_PACKAGE_NAME), 'launch', 'rs_launch.py')

# launch setup
def launch_setup(context, *args, **kwargs):

    # Initialize Arguments
    ur_type = "ur3e" # FIXED TYPE
    safety_limits = LaunchConfiguration("safety_limits")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")
    # General arguments
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    kinematics_params_file = LaunchConfiguration("kinematics_params_file")
    _publish_robot_description_semantic = LaunchConfiguration("publish_robot_description_semantic")
    moveit_config_package = LaunchConfiguration("moveit_config_package")
    moveit_joint_limits_file = LaunchConfiguration("moveit_joint_limits_file")
    moveit_config_file = LaunchConfiguration("moveit_config_file")
    warehouse_sqlite_path = LaunchConfiguration("warehouse_sqlite_path")
    prefix = LaunchConfiguration("prefix")
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_rviz = LaunchConfiguration("launch_rviz")
    launch_servo = LaunchConfiguration("launch_servo")
    namespace = LaunchConfiguration("namespace")

    joint_limit_params = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", "joint_limits.yaml"]
    )
    kinematics_params = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", kinematics_params_file]
    )
    physical_params = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", "physical_parameters.yaml"]
    )
    visual_params = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", "visual_parameters.yaml"]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]),
            " ",
            "robot_ip:=xxx.yyy.zzz.www",
            " ",
            "joint_limit_params:=",
            joint_limit_params,
            " ",
            "kinematics_params:=",
            kinematics_params,
            " ",
            "physical_params:=",
            physical_params,
            " ",
            "visual_params:=",
            visual_params,
            " ",
            "safety_limits:=",
            safety_limits,
            " ",
            "safety_pos_margin:=",
            safety_pos_margin,
            " ",
            "safety_k_position:=",
            safety_k_position,
            " ",
            "name:=",
            "ur",
            " ",
            "ur_type:=",
            ur_type,
            " ",
            "script_filename:=ros_control.urscript",
            " ",
            "input_recipe_filename:=rtde_input_recipe.txt",
            " ",
            "output_recipe_filename:=rtde_output_recipe.txt",
            " ",
            "prefix:=",
            prefix,
            " ",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # MoveIt Configuration
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(moveit_config_package), "srdf", moveit_config_file]
            ),
            " ",
            "name:=",
            # Also ur_type parameter could be used but then the planning group names in yaml
            # configs has to be updated!
            "ur",
            " ",
            "prefix:=",
            prefix,
            " ",
        ]
    )
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_content}

    publish_robot_description_semantic = {
        "publish_robot_description_semantic": _publish_robot_description_semantic
    }

    robot_description_kinematics = PathJoinSubstitution(
        [FindPackageShare(moveit_config_package), "config", "kinematics.yaml"]
    )

    sensors_3d = PathJoinSubstitution(
        [FindPackageShare(moveit_config_package), "config", "sensors_3d.yaml"]
    )

    robot_description_planning = {
        "robot_description_planning": load_yaml(
            str(moveit_config_package.perform(context)),
            os.path.join("config", str(moveit_joint_limits_file.perform(context))),
        )
    }

    # Planning Configuration
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_yaml = load_yaml("ur3e_moveit_config", "config/ompl_planning.yaml")
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    # Trajectory Execution Configuration
    controllers_yaml = load_yaml("ur3e_moveit_config", "config/controllers.yaml")
    # the scaled_joint_trajectory_controller does not work on fake hardware
    change_controllers = context.perform_substitution(use_sim_time)
    if change_controllers == "true":
        controllers_yaml["scaled_joint_trajectory_controller"]["default"] = False
        controllers_yaml["joint_trajectory_controller"]["default"] = True

    moveit_controllers = {
        "moveit_simple_controller_manager": controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
        # Execution time monitoring can be incompatible with the scaled JTC
        "trajectory_execution.execution_duration_monitoring": False,
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

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        namespace=namespace,
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            publish_robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            sensors_3d,
            planning_scene_monitor_parameters,
            {"use_sim_time": use_sim_time},
            warehouse_ros_config,
        ],
        remappings=[
            # namespace (relative) topic
            ("/joint_states", "/ur3e/joint_states"),
        ]
    )

    # rviz with moveit configuration
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(moveit_config_package), "rviz", "view_robot_realsense.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(launch_rviz),
        executable="rviz2",
        name="rviz2_moveit",
        namespace=namespace,
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            robot_description_kinematics,
            robot_description_planning,
            warehouse_ros_config,
            sensors_3d,
            {
                "use_sim_time": use_sim_time,
            },
        ],
    )


    # Static tf between world frame and camera
    # It is necessary to make transformation between world frame and camera frames enable later.
    rs_tf_from_world_publisher_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=[
            str(translation[0]),  # x
            str(translation[1]),  # y
            str(translation[2]),  # z
            str(roll),              # qx (quaternion)
            str(pitch),              # qy (quaternion)
            str(yaw),              # qz (quaternion)
            "world",              # parent frame
            "d435i_link"          # child frame
        ],
    )

    # Servo node for realtime control
    servo_yaml = load_yaml("ur3e_moveit_config", "config/ur_servo.yaml")
    servo_params = {"moveit_servo": servo_yaml}
    servo_node = Node(
        package="moveit_servo",
        condition=IfCondition(launch_servo),
        executable="servo_node_main",
        namespace=namespace,
        parameters=[
            servo_params,
            robot_description,
            robot_description_semantic,
        ],
        output="screen",
        remappings=[
            ("/joint_states", "/ur3e/joint_states"),
        ]
    )

    nodes_to_start = [move_group_node, rviz_node, servo_node, rs_tf_from_world_publisher_node]

    return nodes_to_start


def generate_launch_description():

    declared_arguments = []
    # UR specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_limits",
            default_value="true",
            description="Enables the safety limits controller if true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_pos_margin",
            default_value="0.15",
            description="The margin to lower and upper limits in the safety controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_k_position",
            default_value="20",
            description="k-position factor in the safety controller.",
        )
    )
    # General arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="ur3e_description",
            description="Description package with robot URDF/XACRO files. Usually the argument "
            "is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="ur3e.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "kinematics_params_file",
            default_value="ur3e_mrc_calibration.yaml",
            description="The file name of the calibration configuration of the actual robot used.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "publish_robot_description_semantic",
            default_value="True",
            description="Whether to publish the SRDF description on topic /robot_description_semantic.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_config_package",
            default_value="ur3e_moveit_config",
            description="MoveIt config package with robot SRDF/XACRO files. Usually the argument "
            "is not set, it enables use of a custom moveit config.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_config_file",
            default_value="ur3e.srdf.xacro",
            description="MoveIt SRDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_joint_limits_file",
            default_value="joint_limits.yaml",
            description="MoveIt joint limits that augment or override the values from the URDF robot_description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "warehouse_sqlite_path",
            default_value=os.path.expanduser("~/.ros/warehouse_ros.sqlite"),
            description="Path where the warehouse database should be stored",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Make MoveIt to use simulation time. This is needed for the trajectory planing in simulation.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for "
            "multi-robot setup. If changed than also joint names in the controllers' configuration "
            "have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "namespace",
            default_value="",
            description="Namespace",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz?")
    )
    declared_arguments.append(
        DeclareLaunchArgument("launch_servo", default_value="true", description="Launch Servo?")
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)] + 
                            [IncludeLaunchDescription(
                            PythonLaunchDescriptionSource(REALSENSE_LAUNCH_PATH),
                            launch_arguments={
                                'camera_namespace': params['robot_name'],
                                'camera_name': params['device_type'],
                                'use_sim_time': LaunchConfiguration('use_sim_time'),
                                'serial_no': params['serial_no'],
                                'device_type': params['device_type'],
                                'reconnect_timeout': str(params['reconnect_timeout']),
                                'wait_for_device_timeout': str(params['wait_for_device_timeout']),
                                'world_frame_id': params['world_frame_id'],
                                'enable_rgbd': str(params['enable_rgbd']),
                                'enable_sync': str(params['enable_sync']),
                                'align_depth.enable': str(params['align_depth_enable']), 
                                'enable_color': str(params['enable_color']), 
                                'enable_depth': str(params['enable_depth']),
                                'pointcloud.enable': str(params['enable_pointcloud']),
                                'depth_module.profile': str(params['depth_module_profile']),
                                'rgb_camera.profile': str(params['rgb_camera_profile'])
                            }.items())])
