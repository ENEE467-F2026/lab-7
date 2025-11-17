from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    perception_pkg_share = get_package_share_directory('ur3e_hande_perception')
    use_perception = LaunchConfiguration("use_perception")
    rviz_config = os.path.join(perception_pkg_share, 'rviz', 'pc_test.rviz')
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_rviz = LaunchConfiguration("launch_rviz")


    declared_arguments = [
        DeclareLaunchArgument("use_perception", 
                              default_value="true", 
                              description="Whether to use the perception module to detect object pose. \n " \
                              "Must provide the position of the object to be picked as a launch argument (obj_pos) if false, unless the launch file will throw an error."),

        DeclareLaunchArgument("use_sim_time", default_value="false", description="Use sim time?"),

        DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz?")
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
        condition=IfCondition(launch_rviz),
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
            {"use_sim_time": use_sim_time},
            {"input_topic": "/camera/camera/depth/color/points"},
            {"output_topic": "/filtered_cloud"},
            {"leaf_size": 0.005},
            {"crop_enabled": True},
            {"stop_after_first": False}, # keep processing point clouds since cloud is sparser in real life
            {"crop_bounds": [-0.35, 0.18, -0.5, 0.5, 0.00, 0.9]},  # xmin, xmax, ymin, ymax, zmin, zmax; Point cloud bounds: x[0.00, 1.22], y[0.00, 0.51], z[0.00, 1.50]

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
            {"use_sim_time": use_sim_time},
            {"input_topic": "/filtered_cloud"},
            {"base_frame": "base_link"},
            {"camera_frame": "camera_link"},
            {"stop_after_first_pub": False}, # keep publishing segmentation results
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
                    {"use_sim_time": use_sim_time},
                ],
            )
        ],
    ),
    ])