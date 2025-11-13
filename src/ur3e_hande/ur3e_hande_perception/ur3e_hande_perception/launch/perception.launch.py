from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('ur3e_hande_perception')
    moveit_pkg_share = get_package_share_directory('ur3e_hande_moveit_config')
    use_perception = LaunchConfiguration("use_perception")
    rviz_config = os.path.join(moveit_pkg_share, 'config', 'moveit.rviz')

    declared_arguments = [
        DeclareLaunchArgument("use_perception", default_value="true", description="Whether to use the perception module to detect object pose. \n Must provide the position of the object to be picked as a launch argument (obj_pos) if false, unless the launch file will throw an error."),
    ]

    # Realsense (hardware) launch
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

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config]
    )

    return LaunchDescription(declared_arguments + [
        realsense_node,
        TimerAction(
            period=5.0,
            actions=[rviz_node],
        ),
    ] + [
    # Run perception nodes
    TimerAction(
        period=6.0,
        condition=IfCondition(use_perception),
        actions=[
        Node(
            package="ur3e_hande_perception",
            executable="pc_voxel_filter_node",
            name="pc_voxel_filter_node",
            output="screen",
            parameters=[
            {"use_sim_time": True},
            {"input_topic": "/camera/camera/depth/color/points"},
            {"output_topic": "/filtered_cloud"},
            {"leaf_size": 0.005},
            {"crop_enabled": True},
            # {"crop_bounds": [0.33, 2.00, -0.72, 1.00, -0.50, 0.45]},
            ],
        )
        ]
    ),

    TimerAction(
        period=6.0,
        condition=IfCondition(use_perception),
        actions=[
        Node(
            package="ur3e_hande_perception",
            executable="pc_segmentation_node",
            name="pc_segmentation_node",
            output="screen",
            parameters=[
            {"use_sim_time": True},
            {"input_topic": "/filtered_cloud"}
            ],
        )
        ]
    ),

    # Object Pose Action Server
    TimerAction(
        period=8.0,
        condition=IfCondition(use_perception),
        actions=[
            Node(
                package="ur3e_hande_perception",
                executable="obj_pose_action_server",
                name="object_pose_server",
                output="screen",
                parameters=[
                    {"use_sim_time": True},
                ],
            )
        ],
    ),
    ])