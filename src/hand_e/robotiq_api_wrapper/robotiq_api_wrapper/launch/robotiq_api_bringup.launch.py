from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    ld = LaunchDescription()  
    
    robot_ip = LaunchConfiguration('robot_ip', default='192.168.77.22')

    ld.add_action(Node(
        package="robotiq_api_wrapper",
        executable="gripper_node",
        output="screen",
        parameters=[{"robot_ip": robot_ip}],   
    ))

    return ld