import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():

    use_fake_hw = LaunchConfiguration("use_fake_hardware")
    use_gui = LaunchConfiguration("use_joint_state_gui")

    pkg_share = FindPackageShare("ur3e_hande_description").find("ur3e_hande_description")
    xacro_file = os.path.join(pkg_share, "urdf", "ur3e_hande.urdf.xacro")

    # Convert xacro --> robot_description
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            xacro_file
        ]
    )

    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="both"
    )

    # Joint State Publisher
    joint_state_pub_node = Node(
        condition=IfCondition(use_gui),
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )

    joint_state_pub_fake_node = Node(
        condition=UnlessCondition(use_gui),
        package="joint_state_publisher",
        executable="joint_state_publisher",
    )

    # RViz
    rviz_config = os.path.join(pkg_share, "rviz", "view_robot.rviz")

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        parameters=[robot_description],
        output="screen",
    )

    # Declare args
    return LaunchDescription([
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="true",
            description="Start robot with ros2_control fake hardware"
        ),

        DeclareLaunchArgument(
            "use_joint_state_gui",
            default_value="true",
            description="Use GUI joint sliders"
        ),

        robot_state_publisher_node,
        joint_state_pub_node,
        joint_state_pub_fake_node,
        rviz_node,
    ])
