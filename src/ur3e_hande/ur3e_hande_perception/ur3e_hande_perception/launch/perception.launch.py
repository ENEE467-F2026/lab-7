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
    stop_after_first = LaunchConfiguration("stop_after_first")
    stop_after_first_pub = LaunchConfiguration("stop_after_first_pub")
    crop_enabled = LaunchConfiguration("crop_enabled")
    leaf_size = LaunchConfiguration("leaf_size")


    declared_arguments = [
        DeclareLaunchArgument("use_perception", 
                              default_value="true", 
                              description="Whether to use the perception module to detect object pose. \n " \
                              "Must provide the position of the object to be picked as a launch argument (obj_pos) if false, unless the launch file will throw an error."),

        DeclareLaunchArgument("use_sim_time", default_value="false", description="Use sim time?"),

        DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz?"),
        DeclareLaunchArgument("stop_after_first", default_value="false", description="Stop processing point clouds after the first segmented result?"),
        DeclareLaunchArgument("stop_after_first_pub", default_value="false", description="Stop publishing segmented point cloud after the first result?"),
        DeclareLaunchArgument("crop_enabled", default_value="true", description="Enable cropping of filtered point cloud?" ),
        DeclareLaunchArgument("leaf_size", default_value="0.005", description="Leaf size for voxel grid filter?" ),


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
            {"leaf_size": leaf_size},
            {"crop_enabled": crop_enabled},
            {"stop_after_first": stop_after_first}, # keep processing point clouds since cloud is sparser in real life
            {"crop_bounds": [0.10, 0.65, -0.5, 0.5, 0.00, 0.9]},  # xmin, xmax, ymin, ymax, zmin, zmax; Point cloud bounds: x[0.00, 1.22], y[0.00, 0.51], z[0.00, 1.50]

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
            {"base_frame": "camera_link"}, # change to "world" if launching perception as part of the full arm-gripper manipulation system
            {"camera_frame": "camera_depth_optical_frame"}, # set automatically from point cloud header in pc_segmentation_node.py
            {"stop_after_first_pub": stop_after_first_pub}, # keep publishing segmentation results
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