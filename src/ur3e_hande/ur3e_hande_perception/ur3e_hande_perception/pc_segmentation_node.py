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
ROS 2 Node for tabletop plane segmentation and object clustering.
Assumes input from /filtered_cloud.

Author: Clinton Enwerem
Developed for the course ENEE467: Robotics Projects Laboratory, Fall 2025, University of Maryland, College Park, MD.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
from sensor_msgs_py import point_cloud2
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header
import numpy as np
from sklearn.cluster import DBSCAN
import random
from tf2_ros import Buffer, TransformListener, TransformException
import rclpy.time
from scipy.spatial.transform import Rotation as R

# custom interfaces
from ur3e_hande_planning_interfaces.msg import ObjectMetaData, DetectedObjects

# qos
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class PCSegmentationNode(Node):
    def __init__(self):
        super().__init__('pc_segmentation_node')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.segmentation_done = False

        self.declare_parameter('camera_frame', 'camera_optical_link')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('input_topic', '/filtered_cloud')
        self.declare_parameter('plane_distance_thresh', 0.01)
        self.declare_parameter('dbscan_eps', 0.03)
        self.declare_parameter('dbscan_min_samples', 50)
        self.declare_parameter('surface_thickness', 0.02)
        self.declare_parameter('stop_after_first_pub', False)

        self.input_topic = self.get_parameter('input_topic').value
        self.dist_thresh = self.get_parameter('plane_distance_thresh').value
        self.dbscan_eps = self.get_parameter('dbscan_eps').value
        self.dbscan_min_samples = self.get_parameter('dbscan_min_samples').value
        self.surface_thickness = self.get_parameter('surface_thickness').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.stop_after_first_pub = self.get_parameter('stop_after_first_pub').value

        self.subscription = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self.pointcloud_callback,
            10
        )

        # qos
        plane_qos = QoSProfile(depth=1)
        plane_qos.reliability = ReliabilityPolicy.RELIABLE
        plane_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.surface_pub = self.create_publisher(MarkerArray, '/plane_marker', plane_qos)
        self.object_pub = self.create_publisher(MarkerArray, '/object_markers', 10)
        self.obj_metadata_pub = self.create_publisher(ObjectMetaData, "/object_metadata", 10)
        self.obj_detected_pub = self.create_publisher(DetectedObjects, "/object_detected", 10)

        self.get_logger().info(f"Listening to {self.input_topic} for segmentation...")
        self.get_logger().info("Waiting for TF transform between camera and base_link...")
        rclpy.spin_once(self, timeout_sec=2.0)
        self.tf_ready = False
        try:
            self.tf_camera_to_base = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.camera_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=3.0)
            )
            self.get_logger().info("TF transform successfully acquired.")
            self.tf_ready = True
        except TransformException as e:
            self.get_logger().error(f"Transform lookup failed: {e}")

    def pointcloud_callback(self, msg: PointCloud2):
        if self.segmentation_done and self.stop_after_first_pub:
            return
        try:
            gen = point_cloud2.read_points_numpy(
                msg, field_names=("x", "y", "z"), skip_nans=True, reshape_organized_cloud=True
            )
            if gen is None:
                return
            if gen.size == 0:
                self.get_logger().warning("Received empty point cloud.")
                return

            pts = gen.reshape(-1, 3)

            # Transform from camera frame to base_link frame
            pts = self.transform_points(pts)

            # Proceed with segmentation in the base_link frame
            self.segment_plane_and_objects(pts, msg.header)

        except Exception as e:
            self.get_logger().error(f"Error in pointcloud_callback: {e}")

    def segment_plane_and_objects(self, pts, header):
        """Perform RANSAC plane segmentation and DBSCAN clustering"""
        # Fit plane using RANSAC
        plane_pts, nonplane_pts, plane_eq = self.extract_plane_ransac(pts)
        if plane_pts is None or len(plane_pts) < 100:
            self.get_logger().warning("No dominant plane found.")
            return

        self.publish_plane_marker(plane_pts, header)

        # Cluster objects
        if nonplane_pts is None or len(nonplane_pts) == 0:
            self.get_logger().warning("No points remaining after plane removal")
            return

        labels = self.cluster_objects(nonplane_pts)
        self.publish_object_markers(nonplane_pts, labels, header)

    def extract_plane_ransac(self, pts, max_iterations=200):
        """Simple RANSAC plane segmentation"""
        best_inliers = []
        best_eq = None
        n_pts = len(pts)

        for _ in range(max_iterations):
            ids = np.random.choice(n_pts, 3, replace=False)
            p1, p2, p3 = pts[ids]
            normal = np.cross(p2 - p1, p3 - p1)
            norm = np.linalg.norm(normal)
            if norm < 1e-6:
                continue
            normal /= norm
            d = -np.dot(normal, p1)

            # Distance of all points to plane
            distances = np.abs(pts.dot(normal) + d)
            inliers = np.where(distances < self.dist_thresh)[0]

            if len(inliers) > len(best_inliers):
                best_inliers = inliers
                best_eq = (normal, d)

        if best_eq is None:
            return None, pts, None

        plane_pts = pts[best_inliers]
        mask = np.ones(n_pts, dtype=bool)
        mask[best_inliers] = False
        nonplane_pts = pts[mask]

        self.get_logger().info(
            f"Plane found with {len(plane_pts)} inliers; {len(nonplane_pts)} points remain."
        )
        return plane_pts, nonplane_pts, best_eq

    def transform_points(self, points_cam):
        """Transform Nx3 points from camera to base_link frame"""
        if not self.tf_ready:
            self.get_logger().warning("TF not ready; returning raw points.")
            return points_cam

        t = self.tf_camera_to_base.transform.translation
        q = self.tf_camera_to_base.transform.rotation

        T = np.array([t.x, t.y, t.z])
        R_mat = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()

        points_base = (R_mat @ points_cam.T).T + T
        return points_base

    def cluster_objects(self, pts):
        """Cluster non-plane points with DBSCAN."""
        clustering = DBSCAN(
            eps=self.dbscan_eps, min_samples=self.dbscan_min_samples
        ).fit(pts)
        return clustering.labels_

    
    def publish_plane_marker(self, plane_pts, header):
        """Publish the detected plane as a semi-transparent cube marker."""
        centroid = np.mean(plane_pts, axis=0)
        min_bounds = np.min(plane_pts, axis=0)        # [min_x, min_y, min_z]
        max_bounds = np.max(plane_pts, axis=0)        # [max_x, max_y, max_z]
        length, width = max_bounds[0] - min_bounds[0], max_bounds[1] - min_bounds[1]

        marker = Marker()
        marker.header = Header(frame_id=self.base_frame)
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.id = 0
        marker.pose.position.x = float(centroid[0])
        marker.pose.position.y = float(centroid[1])
        marker.pose.position.z = float(centroid[2] - self.surface_thickness / 2.0)
        marker.pose.orientation.w = 1.0
        marker.scale.x = float(length)
        marker.scale.y = float(width)
        marker.scale.z = float(self.surface_thickness)
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.4
        marker.lifetime = rclpy.duration.Duration(seconds=0).to_msg()

        marker_array = MarkerArray()
        marker_array.markers.append(marker)
        self.surface_pub.publish(marker_array)

    
    def publish_object_markers(self, pts, labels, header):
        """Publish DBSCAN clusters as colored cube markers."""
        unique_labels = set(labels) - {-1}
        markers = []

        for cluster_id in unique_labels:
            cluster_pts = pts[labels == cluster_id]
            centroid = np.mean(cluster_pts, axis=0)
            min_bounds = np.min(cluster_pts, axis=0)
            max_bounds = np.max(cluster_pts, axis=0)
            dims = max_bounds - min_bounds

            r, g, b = [random.random() for _ in range(3)]
            marker = Marker()
            marker.header = Header(frame_id=self.base_frame)
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.id = int(cluster_id)
            marker.pose.position.x = float(centroid[0])
            marker.pose.position.y = float(centroid[1])
            marker.pose.position.z = float(centroid[2])
            marker.pose.orientation.w = 1.0
            marker.scale.x = float(dims[0])
            marker.scale.y = float(dims[1])
            marker.scale.z = float(dims[2])
            marker.color.r = r
            marker.color.g = g
            marker.color.b = b
            marker.color.a = 0.7
            marker.lifetime = rclpy.duration.Duration(seconds=0).to_msg()
            markers.append(marker)

            self.publish_object_metadata(cluster_id, centroid, dims, min_bounds, max_bounds)

        marker_array = MarkerArray(markers=markers)
        self.object_pub.publish(marker_array)
        
        if not self.segmentation_done and self.stop_after_first_pub:
            self.get_logger().info(f"Published {len(unique_labels)} object clusters.")
            self.get_logger().info("Segmentation complete; stopping pointcloud subscriber.")
            self.segmentation_done = True

    def publish_object_metadata(self, cluster_id: int, centroid, dims, min_bounds, max_bounds):
        """Publish ObjectMetaData and DetectedObjects messages."""

        # self.get_logger().warning(f" {cluster_id}\n Centroid: {centroid}\n Dims: {dims}\n Min bounds: {min_bounds}\n Max bounds: {max_bounds}")
        # ObjectMetaData
        meta_msg = ObjectMetaData()
        meta_msg.id = int(cluster_id)

        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = str(self.base_frame)
        pose_msg.pose.position.x = float(centroid[0]) 
        pose_msg.pose.position.y = float(centroid[1])
        pose_msg.pose.position.z = float(centroid[2])
        pose_msg.pose.orientation.w = 1.0  # the object is upright by default
        meta_msg.object_pose = pose_msg

        # dimensions and centroid
        meta_msg.dimensions = [float(x) for x in dims.tolist()]           # [h, w, t]
        meta_msg.centroid = [float(c) for c in centroid.tolist()]         # [x, y, z]

        # bounds
        min_x, max_x = float(min_bounds[0]), float(max_bounds[0])
        min_y, max_y = float(min_bounds[1]), float(max_bounds[1])
        meta_msg.bounds = [float(b) for b in [min_x, max_x, min_y, max_y]]

        # DetectedObjects 
        det_msg = DetectedObjects()
        det_msg.object_id = int(cluster_id)
        det_msg.position.x = float(centroid[0])
        det_msg.position.y = float(centroid[1])
        det_msg.position.z = float(centroid[2])
        det_msg.height = float(dims[0])
        det_msg.width = float(dims[1])
        det_msg.thickness = float(dims[2])

        # publish data
        self.obj_metadata_pub.publish(meta_msg)
        self.obj_detected_pub.publish(det_msg)

        self.get_logger().info(
            f"Published metadata for object {cluster_id} at {centroid}, dims={dims}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = PCSegmentationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()