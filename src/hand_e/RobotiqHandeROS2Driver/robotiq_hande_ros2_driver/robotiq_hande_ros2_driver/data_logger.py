#!/usr/bin/env python3

"""
Each pickle file contains a list of dictionaries of the following form:

{
  'timestamp': 1723452891234567890,
  'task_id': 'pick_and_place',
  'rgb_image': <np.ndarray>,
  'depth_image': <np.ndarray>,
  'joint_state': {
      'name': [...],
      'position': [...],
      'velocity': [...],
      'effort': [...],
  },
  'ee_joint_state': {
      'position': [...],
      'orientation': [...],
  },
}

"""
import rclpy
from rclpy.node import Node
import numpy as np
import pickle
import os
from sensor_msgs.msg import JointState, Image, PointCloud2
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
from builtin_interfaces.msg import Time

import cv2
from datetime import datetime

# transforms
from tf2_ros import LookupException, TransformException, ConnectivityException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros.transform_broadcaster import TransformBroadcaster
from scipy.spatial.transform import Rotation as R

# o3d
import open3d as o3d

# type safety
from typing import Union

class DataLoggerNode(Node):
    def __init__(self):
        super().__init__('data_logger')
        self.bridge = CvBridge()

        self.task_id = self.declare_parameter('task_id', 'pick_and_place').get_parameter_value().string_value
        self.output_dir = self.declare_parameter('output_dir', value='/media/callab-a/Bagel/data_logs').get_parameter_value().string_value
        os.makedirs(self.output_dir, exist_ok=True)

        self.image_rgb = None
        self.depth_image = None
        self.pcd = None
        self.joint_state = None
        self.ee_joint_state = None

        # tf
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # throttle logs for sanity
        self.time_btw_logs = 5e9
        self.last_log_time = {
            'rgb': 0,
            'depth': 0,
            'pc': 0,
            'ur': 0,
            'ee': 0,
        }

        # topics to listen to
        self.create_subscription(Image, 'camera/camera/color/image_raw', self.rgb_cb, 10)
        self.create_subscription(Image, 'camera/camera/depth/image_rect_raw', self.depth_cb, 10)
        self.create_subscription(PointCloud2, '/camera/camera/depth/color/points', self.pc_cb, 10)
        self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)
        self.create_subscription(JointState, '/hande/joint_states', self.ee_cb, 10)
        self.create_timer(0.1, self.record_data)  # 10 Hz

        self.episode_data = []
        self.episode_id = datetime.now().strftime("%Y%m%d_%H%M%S")

        self.get_logger().info(f"Logger started for task: {self.task_id}")
    
    def quaternion_to_rotation_matrix(self, quat: np.ndarray) -> np.ndarray:
        return R.from_quat([quat[0], quat[1], quat[2], quat[3]]).as_matrix()

    def ros_msg_to_point_cloud(self, msg: PointCloud2) -> Union[o3d.geometry.PointCloud, None]:
        """Helper function for converting a ROS PointCloud2 message to an o3d point cloud"""
        try:
            transform = self.tf_buffer.lookup_transform('base_link',
                                                        msg.header.frame_id,
                                                        rclpy.time.Time(),
                                                        timeout=rclpy.time.Duration(seconds=1.0))
            translation = np.array([transform.transform.translation.x,
                                    transform.transform.translation.y,
                                    transform.transform.translation.z])
            rotation_quaternion = np.array([transform.transform.rotation.x,
                                            transform.transform.rotation.y,
                                            transform.transform.rotation.z,
                                            transform.transform.rotation.w])

            # convert quaternion to rotation matrix
            rotation_matrix = self.quaternion_to_rotation_matrix(rotation_quaternion)

            # convert PointCloud2 msg to numpy array
            point_step = msg.point_step
            num_points = len(msg.data) // point_step
            points = []
            for i in range(num_points):
                start_index = i * point_step
                x_bytes = msg.data[start_index:start_index + 4]
                y_bytes = msg.data[start_index + 4:start_index + 8]
                z_bytes = msg.data[start_index + 8:start_index + 12]
                x = np.frombuffer(x_bytes, dtype=np.float32)[0]
                y = np.frombuffer(y_bytes, dtype=np.float32)[0]
                z = np.frombuffer(z_bytes, dtype=np.float32)[0]
                point = np.array([x, y, z])

                # apply the rotation to the point
                rotated_point = np.dot(rotation_matrix, point)

                # apply the translation to the rotated point to get its position relative to the ur's base_link frame
                relative_point = rotated_point + translation

                points.append(relative_point)

            data = np.array(points, dtype=np.float32)
            assert data.shape[1] == 3, "Number of fields must be 3"
            cloud = o3d.geometry.PointCloud()
            cloud.points = o3d.utility.Vector3dVector(data)
            return cloud

        except (TransformException, ConnectivityException) as e:
            self.get_logger().error(f"Transform lookup failed: {e}")
        except Exception as e:
            self.get_logger().error(f"Error in from_ros_msg: {e}")
            return None
        
    def rgb_cb(self, msg):
        try:
            self.image_rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            now = self.get_clock().now().nanoseconds
            if now - self.last_log_time['rgb'] > self.time_btw_logs:  
                self.get_logger().info(f"{self.task_id}: Logging RGB image stream from camera node.")
                self.last_log_time['rgb'] = now
        except Exception as e:
            self.get_logger().error(f"RGB image error: {e}")

    def depth_cb(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg)
            now = self.get_clock().now().nanoseconds
            if now - self.last_log_time['depth'] > self.time_btw_logs:  
                self.get_logger().info(f"{self.task_id}: Logging Depth image stream from camera node.")
                self.last_log_time['depth'] = now

        except Exception as e:
            self.get_logger().error(f"Depth image error: {e}")

    def pc_cb(self, msg):
        try:
            self.pcd = self.ros_msg_to_point_cloud(msg)
            now = self.get_clock().now().nanoseconds
            if now - self.last_log_time['pc'] > self.time_btw_logs:  
                self.get_logger().info(f"{self.task_id}: Logging point cloud stream from camera node.")
                self.last_log_time['pc'] = now

        except Exception as e:
            self.get_logger().error(f"Point cloud error: {e}")

    def joint_cb(self, msg: JointState):
        try:
            self.joint_state = {
                'name': msg.name,
                'position': list(msg.position),
                'velocity': list(msg.velocity),
                'effort': list(msg.effort),
            }
            now = self.get_clock().now().nanoseconds
            if now - self.last_log_time['ur'] > self.time_btw_logs:  
                self.get_logger().info(f"{self.task_id}: Logging UR joint states.")
                self.last_log_time['ur'] = now
            
        except Exception as e:
            self.get_logger().error(f"Error logging UR joint states: {e}")

    def ee_cb(self, msg: JointState):
        try:
            self.ee_joint_state = {
                'position': [msg.position],
                'velocity': [msg.velocity],
                'effort': [msg.effort],
                'name': [msg.name]
            }
            now = self.get_clock().now().nanoseconds
            if now - self.last_log_time['ee'] > self.time_btw_logs:  
                self.get_logger().info(f"{self.task_id}: Logging end effector joint state.")
                self.last_log_time['ee'] = now
        except Exception as e:
            self.get_logger().error(f"Error logging Hand-E joint state: {e}")

    def record_data(self):
        if self.joint_state is None or self.image_rgb is None or self.ee_joint_state is None:
            return
        
        timestamp = self.get_clock().now().nanoseconds
        ply_filename = f"cloud_{timestamp}_{self.task_id}_{self.episode_id}.ply"
        ply_path = os.path.join(self.output_dir, 'ply_files', ply_filename)
        o3d.io.write_point_cloud(ply_path, self.pcd)
        frame_data = {
            'timestamp': timestamp,
            'task_id': self.task_id,
            'rgb_image': self.image_rgb.copy(),
            'depth_image': self.depth_image.copy() if self.depth_image is not None else None,
            'point_cloud': np.asarray(self.pcd.points) if self.pcd is not None else None,
            'joint_state': self.joint_state,
            'ee_joint_state': self.ee_joint_state,
        }

        self.episode_data.append(frame_data)

    def save_episode(self):
        path = os.path.join(self.output_dir, f'{self.task_id}_{self.episode_id}.pkl')
        with open(path, 'wb') as f:
            pickle.dump(self.episode_data, f)
        self.get_logger().info(f"Saved episode with {len(self.episode_data)} frames to: {path}")
        self.episode_data.clear()

    def destroy_node(self):
        self.save_episode()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    logger_node = DataLoggerNode()
    try:
        rclpy.spin(logger_node)
    except KeyboardInterrupt:
        pass
    finally:
        logger_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
