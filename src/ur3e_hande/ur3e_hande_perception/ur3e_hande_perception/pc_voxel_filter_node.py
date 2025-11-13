#!/usr/bin/env python3

# Lab 7: Autonomous Manipulation with ROS 2 on the Real UR3e-Hand-E Robot
# Copyright (C) 2025 Clinton Enwerem

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#     http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
ROS 2 Node for lightweight voxel downsampling and cropping of live point clouds.

Author: Clinton Enwerem
Developed for the course ENEE467: Robotics Projects Laboratory, Fall 2025, University of Maryland, College Park, MD.
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
        self.declare_parameter('crop_bounds', [0.33, 2.00, -0.72, 1.00, -0.50, 0.45])  
        self.declare_parameter("stop_after_first", True)
        # [xmin, xmax, ymin, ymax, zmin, zmax] x[0.33, 2.00], y[-0.72, 1.00], z[-0.50, 0.45]

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.leaf_size = self.get_parameter('leaf_size').value
        self.crop_enabled = self.get_parameter('crop_enabled').value
        self.crop_bounds = self.get_parameter('crop_bounds').value

        self.stop_after_first = self.get_parameter("stop_after_first").value
        self.done = False

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
        if self.stop_after_first and self.done:
            return
        try:
            # Read all XYZ points as list of tuples (x, y, z)
            # dtype({'names': ['x', 'y', 'z'], 'formats': ['<f4', '<f4', '<f4'], 'offsets': [0, 4, 8], 'itemsize': 24}) 
            ###
            gen = point_cloud2.read_points_numpy(msg, field_names=("x", "y", "z"), skip_nans=True, reshape_organized_cloud=True)
            # data = np.fromiter(gen)
            if gen is None:
                return
            
            if gen.size == 0:
                self.get_logger().warning("Received empty point cloud.")
                return

            # Convert structured array to plain (N,3)
            # gen is WxHx3
            points = gen.reshape(-1, 3)

            # Remove NaN/Inf values
            mask_finite = np.isfinite(points).all(axis=1)
            points = points[mask_finite]

            mask_reasonable = (
                (points[:, 0] > -1.0) & (points[:, 0] < 2.0) &
                (points[:, 1] > -1.0) & (points[:, 1] < 1.0) &
                (points[:, 2] > -0.5) & (points[:, 2] < 1.5)
            )
            points = points[mask_reasonable]

            if points.size == 0:
                self.get_logger().warning("All points invalid (NaN/Inf). Skipping frame.")
                return

            if points.ndim != 2 or points.shape[1] != 3:
                self.get_logger().warning(f"Unexpected point array shape: {points.shape}")
                return

            # self.get_logger().info(
            #     f"Point cloud bounds: x[{points[:,0].min():.2f}, {points[:,0].max():.2f}], "
            #     f"y[{points[:,1].min():.2f}, {points[:,1].max():.2f}], "
            #     f"z[{points[:,2].min():.2f}, {points[:,2].max():.2f}]"
            # )

            # Cropping
            if self.crop_enabled:
                xmin, xmax, ymin, ymax, zmin, zmax = self.crop_bounds
                mask = (
                    (points[:, 0] > xmin) & (points[:, 0] < xmax) &
                    (points[:, 1] > ymin) & (points[:, 1] < ymax) &
                    (points[:, 2] > zmin) & (points[:, 2] < zmax)
                )
                points = points[mask]

            if points.size == 0:
                self.get_logger().warning("All points filtered out after cropping.")
                return

            # Downsample using voxel grid
            voxel_indices = np.floor(points / self.leaf_size)
            _, unique_indices = np.unique(voxel_indices, axis=0, return_index=True)
            filtered_points = points[unique_indices]

            # Publish filtered cloud
            header = Header()
            header.stamp = msg.header.stamp
            header.frame_id = msg.header.frame_id
            cloud_msg = point_cloud2.create_cloud_xyz32(header, filtered_points.tolist())
            self.publisher.publish(cloud_msg)

            self.get_logger().info(
                f"Publishing filtered cloud (~{filtered_points.shape[0]} points per frame)"
            )
            self.done = True

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
