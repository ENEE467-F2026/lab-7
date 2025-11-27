from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    perception_pkg_share = get_package_share_directory('ur3e_hande_perception')
    use_perception = LaunchConfiguration("use_perception")
    rviz_config = os.path.join(perception_pkg_share, 'rviz', 'pc_test.rviz')
    use_sim_time = LaunchConfiguration("use_sim_time")
    base_frame = LaunchConfiguration("base_frame")
    camera_frame = LaunchConfiguration("camera_frame")
    launch_rviz = LaunchConfiguration("launch_rviz")
    stop_after_first = LaunchConfiguration("stop_after_first")
    stop_after_first_pub = LaunchConfiguration("stop_after_first_pub")
    crop_enabled = LaunchConfiguration("crop_enabled")
    leaf_size = LaunchConfiguration("leaf_size")
    pc_seg_input_topic = LaunchConfiguration("pc_seg_input_topic")
    pc_topic = LaunchConfiguration("pc_topic")


    declared_arguments = [
        DeclareLaunchArgument("use_perception", 
                              default_value="true", 
                              description="Whether to use the perception module to detect object pose. \n " \
                              "Must provide the position of the object to be picked as a launch argument (obj_pos) if false, unless the launch file will throw an error."),

        DeclareLaunchArgument("use_sim_time", default_value="false", description="Use sim time?"),

        DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz?"),
        DeclareLaunchArgument("stop_after_first", default_value="false", description="Stop processing point clouds after the first segmented result?"),
        DeclareLaunchArgument("stop_after_first_pub", default_value="false", description="Stop publishing segmented point cloud after the first result? \n " \
        "If set to true, the point cloud segmentation node will keep publishing segmentation results."),
        DeclareLaunchArgument("crop_enabled", default_value="true", description="Enable cropping of filtered point cloud?" ),
        DeclareLaunchArgument("leaf_size", default_value="0.005", description="Leaf size for voxel grid filter?" ),
        DeclareLaunchArgument("base_frame", default_value="camera_link", description="Name of frame in which the positions of detected objects is expressed.\n" \
        "Use 'base_link' if launching this launch file as part of a full manipulation system."),
        DeclareLaunchArgument("camera_frame", default_value="camera_depth_optical_frame", description="Name of the camera frame."),
        DeclareLaunchArgument("pc_topic", default_value="/camera/camera/depth/color/points", description="Topic from which to subscribe to the raw point cloud."),
        DeclareLaunchArgument("pc_seg_input_topic", default_value="/filtered_cloud", description="Topic from which to subscribe to the raw point cloud."),


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

    # static tf broadcaster
    static_tf_node_1 = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '--x', '0.0', '--y', '0.0', '--z', '0.7',
                '--yaw', '0', '--pitch', '0.0', '--roll',
                '0', '--frame-id', 'world', '--child-frame-id', 'base_link']
        )
    
    # static tf broadcaster
    static_tf_node_2 = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            condition=IfCondition(
                PythonExpression(["'", base_frame, "' == 'base_link'"])
            ),
            arguments=[
                '--x', '0.58', '--y', '0.533', '--z', '0.467',
                '--yaw', '-3.14', '--pitch', '0.32', '--roll',
                '0', '--frame-id', 'base_link', '--child-frame-id', 'camera_link']
        )

    return LaunchDescription(declared_arguments + [
        realsense_node,
        static_tf_node_1,
        static_tf_node_2,
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
            {"input_topic": pc_topic},
            {"output_topic": pc_seg_input_topic},
            {"leaf_size": leaf_size},
            {"crop_enabled": crop_enabled},
            {"stop_after_first": stop_after_first}, 
            {"crop_bounds": [-0.4, 0.3, 0.0, 0.65, 0.00, 1.2]},  # xmin, xmax, ymin, ymax, zmin, zmax; Point cloud bounds: x[0.00, 1.22], y[0.00, 0.51], z[0.00, 1.50]

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
            {"input_topic": pc_seg_input_topic},
            {"base_frame": base_frame}, 
            {"camera_frame": camera_frame}, # set automatically from point cloud header in pc_segmentation_node.py
            {"stop_after_first_pub": stop_after_first_pub}, 
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