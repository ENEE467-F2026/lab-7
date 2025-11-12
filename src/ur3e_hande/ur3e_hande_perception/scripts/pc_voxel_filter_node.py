#!/usr/bin/env python3
"""
ROS 2 Node for lightweight voxel downsampling and cropping of live point clouds.
Author: Clinton Enwerem
"""

import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

class PCVoxelFilterNode(Node):
    def __init__(self):
        super().__init__('pc_voxel_filter_node')

        # Declare parameters
        self.declare_parameter('input_topic', '/rgbd_camera/points')
        self.declare_parameter('output_topic', '/filtered_cloud')
        self.declare_parameter('leaf_size', 0.005)
        self.declare_parameter('crop_enabled', True)
        self.declare_parameter('crop_bounds', [-0.5, 0.5, -0.5, 0.5, 0.0, 1.0])  # [xmin, xmax, ymin, ymax, zmin, zmax]

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.leaf_size = self.get_parameter('leaf_size').value
        self.crop_enabled = self.get_parameter('crop_enabled').value
        self.crop_bounds = self.get_parameter('crop_bounds').value

        # Subscriber and publisher
        self.subscription = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self.pointcloud_callback,
            10
        )
        self.publisher = self.create_publisher(PointCloud2, self.output_topic, 10)
        self.get_logger().info(f"Listening to {self.input_topic}, publishing filtered cloud on {self.output_topic}")

    def pointcloud_callback(self, msg):
        try:
            # Convert generator to list first
            points_list = list(point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
            if len(points_list) == 0:
                self.get_logger().warn_once("Received empty point cloud.")
                return

            # Convert to numeric array
            points = np.stack([points_list['x'], points_list['y'], points_list['z']], axis=-1)

            # Optional crop
            if self.crop_enabled:
                xmin, xmax, ymin, ymax, zmin, zmax = self.crop_bounds
                mask = (
                    (points[:, 0] > xmin) & (points[:, 0] < xmax) &
                    (points[:, 1] > ymin) & (points[:, 1] < ymax) &
                    (points[:, 2] > zmin) & (points[:, 2] < zmax)
                )
                points = points[mask]

            if points.size == 0:
                self.get_logger().warn_once("All points filtered out after cropping.")
                return

            # Downsample using voxel grid
            voxel_indices = np.floor(points / self.leaf_size)
            _, unique_indices = np.unique(voxel_indices, axis=0, return_index=True)
            filtered_points = points[unique_indices]

            # Publish
            header = Header()
            header.stamp = msg.header.stamp
            header.frame_id = msg.header.frame_id
            cloud_msg = point_cloud2.create_cloud_xyz32(header, filtered_points.tolist())
            self.publisher.publish(cloud_msg)

            self.get_logger().info_once(
                f"Publishing filtered cloud (~{filtered_points.shape[0]} points per frame)"
            )

        except Exception as e:
            self.get_logger().error(f"Error in pointcloud_callback: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = PCVoxelFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
