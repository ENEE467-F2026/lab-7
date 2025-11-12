from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    urdf_file = PathJoinSubstitution([
        FindPackageShare("ur3e_hande_description"),
        "urdf",
        "ur3e_hande.urdf.xacro"
    ])

    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{
                "robot_description": Command([
                    PathJoinSubstitution([FindExecutable(name="xacro")]),
                    " ",
                    urdf_file
                ])
            }],
            output="screen"
        )
    ])
