# Copyright (c) 2021 PickNik, Inc.

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch_ros.parameter_descriptions import ParameterValue, ParameterFile
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    FindExecutable,
    Command,
)

try:
    from ur3e_moveit_config.launch_common import load_yaml
    MOVEIT_AVAILABLE = True
except ImportError:
    MOVEIT_AVAILABLE = False
    def load_yaml(package, file_path):
        return {}

def launch_setup(context, *args, **kwargs):
    robot_ip = LaunchConfiguration("robot_ip")
    launch_rviz = LaunchConfiguration("launch_rviz")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    fake_sensor_commands = LaunchConfiguration("fake_sensor_commands")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")
    activate_joint_controller = LaunchConfiguration("activate_joint_controller")
    ur_description_package = LaunchConfiguration("ur_description_package")
    description_file = LaunchConfiguration("description_file")
    use_moveit = LaunchConfiguration("use_moveit")
    safety_limits = LaunchConfiguration("safety_limits")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")
    moveit_config_package = LaunchConfiguration("moveit_config_package")
    warehouse_sqlite_path = LaunchConfiguration("warehouse_sqlite_path")
    launch_servo = LaunchConfiguration("launch_servo")
    
    kinematic_params_file = PathJoinSubstitution(
        [FindPackageShare(ur_description_package), "config", LaunchConfiguration("kinematics_params_file")]
    )

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([FindPackageShare("ur3e_description"), "urdf", "ur3e.urdf.xacro"]),
        " robot_ip:=", robot_ip,
        " safety_limits:=", safety_limits,
        " safety_pos_margin:=", safety_pos_margin,
        " safety_k_position:=", safety_k_position,
    ])
    
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("ur_robot_driver"),
                "launch",
                "ur_control.launch.py"
            ]),
        ]),
        launch_arguments={
            "ur_type": "ur3e",
            "launch_rviz": "false",
            "launch_robot_state_publisher": "true",
            "robot_ip": robot_ip,
            "use_fake_hardware": use_fake_hardware,
            "fake_sensor_commands": fake_sensor_commands,
            "initial_joint_controller": initial_joint_controller,
            "activate_joint_controller": activate_joint_controller,
            "description_package": ur_description_package,
            "description_file": description_file,
            "kinematics_params_file": kinematic_params_file,
        }.items(),
    )

    nodes_to_start = [
        robot_state_publisher_node,
        base_launch,
    ]

    use_moveit_bool = use_moveit.perform(context) == "true"
    
    if use_moveit_bool and MOVEIT_AVAILABLE:
        robot_description_semantic_content = Command([
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(moveit_config_package), "config", "ur3e_hande.srdf.xacro"]),
            " ",
            "name:=ur",
        ])
        robot_description_semantic = {
            "robot_description_semantic": ParameterValue(robot_description_semantic_content, value_type=str)
        }

        robot_description_kinematics = PathJoinSubstitution(
            [FindPackageShare(moveit_config_package), "config", "kinematics.yaml"]
        )

        robot_description_planning = {
            "robot_description_planning": load_yaml(
                moveit_config_package.perform(context),
                "config/joint_limits.yaml",
            )
        }

        ompl_planning_pipeline_config = {
            "move_group": {
                "planning_plugin": "ompl_interface/OMPLPlanner",
                "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
                "start_state_max_bounds_error": 0.1,
            }
        }
        
        try:
            ompl_planning_yaml = load_yaml("ur3e_moveit_config", "config/ompl_planning.yaml")
            ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)
        except:
            pass

        try:
            moveit_controllers = load_yaml("ur3e_hande_moveit_config", "config/moveit_controllers.yaml")
        except:
            moveit_controllers = {
                "moveit_simple_controller_manager": {
                    "controller_names": ["scaled_joint_trajectory_controller", "joint_trajectory_controller"],
                    "scaled_joint_trajectory_controller": {
                        "type": "FollowJointTrajectory",
                        "action_ns": "follow_joint_trajectory",
                        "default": True,
                        "joints": ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", 
                                 "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
                    },
                    "joint_trajectory_controller": {
                        "type": "FollowJointTrajectory",
                        "action_ns": "follow_joint_trajectory",
                        "default": False,
                        "joints": ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", 
                                 "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
                    }
                },
                "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
            }

        trajectory_execution = {
            "moveit_manage_controllers": False,
            "trajectory_execution.allowed_execution_duration_scaling": 1.2,
            "trajectory_execution.allowed_goal_duration_margin": 0.5,
            "trajectory_execution.allowed_start_tolerance": 0.01,
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

        move_group_node = Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[
                robot_description,
                robot_description_semantic,
                robot_description_kinematics,
                robot_description_planning,
                ompl_planning_pipeline_config,
                trajectory_execution,
                moveit_controllers,
                planning_scene_monitor_parameters,
                warehouse_ros_config,
                {"use_sim_time": use_fake_hardware},
            ]
        )

        rviz_config_file = PathJoinSubstitution(
            [FindPackageShare(moveit_config_package), "config", "moveit.rviz"]
        )
        rviz_moveit_node = Node(
            package="rviz2",
            condition=IfCondition(launch_rviz),
            executable="rviz2",
            name="rviz2_moveit",
            output="log",
            arguments=["-d", rviz_config_file],
            parameters=[
                robot_description,
                robot_description_semantic,
                ompl_planning_pipeline_config,
                robot_description_kinematics,
                robot_description_planning,
                warehouse_ros_config,
                {"use_sim_time": use_fake_hardware},
            ],
        )

        servo_node = Node(
            package="moveit_servo",
            condition=IfCondition(launch_servo),
            executable="servo_node_main",
            parameters=[
                robot_description,
                robot_description_semantic,
                {"use_sim_time": use_fake_hardware},
            ],
            output="screen"
        )

        nodes_to_start.extend([
            move_group_node,
            rviz_moveit_node,
            servo_node,
        ])

    elif use_moveit_bool and not MOVEIT_AVAILABLE:
        print("WARNING: MoveIt requested but ur3e_moveit_config not available. Starting hardware only.")

    return nodes_to_start

def generate_launch_description():
    declared_arguments = []
    
    declared_arguments.extend([
        DeclareLaunchArgument(
            "robot_ip",
            default_value="192.168.77.22",
            description="IP address by which the robot can be reached.",
        ),
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="false",
            description="Launch RViz?"
        ),
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false", 
            description="Start robot with fake hardware mirroring command to its states.",
        ),
        DeclareLaunchArgument(
            "fake_sensor_commands",
            default_value="false",
            description="Enable fake command interfaces for sensors used for simple simulations.",
        ),
        DeclareLaunchArgument(
            "initial_joint_controller",
            default_value="scaled_joint_trajectory_controller",
            description="Initially loaded robot controller.",
        ),
        DeclareLaunchArgument(
            "activate_joint_controller",
            default_value="true",
            description="Activate loaded joint controller.",
        ),
        DeclareLaunchArgument(
            "ur_description_package",
            default_value="ur3e_description",
            description="Description package with robot URDF/XACRO files.",
        ),
        DeclareLaunchArgument(
            "description_file",
            default_value="ur3e.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        ),
        DeclareLaunchArgument(
            "kinematics_params_file",
            default_value="ur3e_mrc_calibration.yaml",
            description="The file name of the calibration configuration of the actual robot used.",
        ),
    ])

    declared_arguments.extend([
        DeclareLaunchArgument(
            "use_moveit",
            default_value="false",
            description="Start MoveIt planning and control nodes.",
        ),
        DeclareLaunchArgument(
            "safety_limits",
            default_value="true",
            description="Enables the safety limits controller if true.",
        ),
        DeclareLaunchArgument(
            "safety_pos_margin",
            default_value="0.15",
            description="The margin to lower and upper limits in the safety controller.",
        ),
        DeclareLaunchArgument(
            "safety_k_position",
            default_value="20",
            description="k-position factor in the safety controller.",
        ),
        DeclareLaunchArgument(
            "moveit_config_package",
            default_value="ur3e_hande_moveit_config",
            description="MoveIt config package with robot SRDF/XACRO files.",
        ),
        DeclareLaunchArgument(
            "warehouse_sqlite_path",
            default_value=os.path.expanduser("~/.ros/warehouse_ros.sqlite"),
            description="Path where the warehouse database should be stored",
        ),
        DeclareLaunchArgument(
            "launch_servo",
            default_value="false",
            description="Launch MoveIt Servo for real-time control.",
        ),
    ])

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
