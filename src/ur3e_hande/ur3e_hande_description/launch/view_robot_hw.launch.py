import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Configurable Launch Args
    use_fake_hw = LaunchConfiguration("use_fake_hardware")
    create_socat_tty = LaunchConfiguration("create_socat_tty")

    # Locate package
    pkg_share = FindPackageShare("ur3e_hande_description").find("ur3e_hande_description")
    xacro_file = os.path.join(pkg_share, "urdf", "ur3e_hande_hw.urdf.xacro")

    # Convert xacro -> URDF
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        xacro_file,
        " use_fake_hardware:=", use_fake_hw,
        " import_ur3e:=true",
        " import_hande:=true",
        " import_camera:=true",
        " hande_name:=robotiq_hande",
        " tty_port:=/tmp/ttyUR",
        " create_socat_tty:=", create_socat_tty
    ])

    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # rsp
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="both"
    )

    # ros2_control Node 
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            os.path.join(pkg_share, "config", "controllers.yaml"),
        ],
        output="screen",
    )

    # Load controllers
    load_controllers = []
    for controller in [
        "joint_state_broadcaster",
        "scaled_joint_trajectory_controller",
        "gripper_action_controller",
    ]:
        load_controllers.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller, "-c", "/controller_manager"],
                output="screen",
            )
        )

    # RViz
    rviz_config = os.path.join(pkg_share, "rviz", "view_robot_hw.rviz")

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        parameters=[robot_description],
        output="screen"
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",  
            description="Set to false for real robot"
        ),

        DeclareLaunchArgument(
            "use_tool_comm",
            default_value="true",
            description="Enable socat Forwarder for Robotiq Modbus"
        ),

        rsp_node,
        ros2_control_node,
        *load_controllers,
        rviz_node,
    ])
