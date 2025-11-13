from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('ur3e_hande_perception')
    rviz_config = os.path.join(pkg_share, 'rviz', 'view_detections.rviz')
    script_path = os.path.join(pkg_share, 'scripts', 'yolo_pc_pose_estimation.py')

    # Realsense
    realsense_node = ExecuteProcess(
        cmd=[
            'ros2', 'launch', 'realsense2_camera', 'rs_launch.py',
            'depth_module.depth_profile:=640x480x30',
            'rgb_camera.color_profile:=640x480x30',
            'pointcloud.enable:=true',
            'align_depth.enable:=true',
            'pointcloud.ordered_pc:=true',
            'enable_infra:=true',
            'enable_infra1:=true',
            'enable_infra2:=true',
            'depth_module.infra_profile:=640x480x30'
        ],
        output='log'
    )

    # YOLO
    yolo_node = Node(
        package='ur3e_hande_perception',
        executable=script_path,
        name='yolo_pc_pose_estimation',
        output='screen',
        emulate_tty=True
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config]
    )

    return LaunchDescription([
        realsense_node,
        yolo_node,
        rviz_node
    ])