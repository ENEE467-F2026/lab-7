#!/usr/bin/env python3
import os
import yaml
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from scipy.spatial.transform import Rotation as R


class StaticCameraTFPublisher(Node):

    def __init__(self):
        super().__init__("camera_tf_publisher")

        # Declare parameter
        camera_calib_file = self.declare_parameter(
            "camera_calib_file", ""
        ).value

        camera_mount_frame = self.declare_parameter(
            "camera_mount_frame", "camera_mount"
        ).value

        if not camera_calib_file or not os.path.exists(str(camera_calib_file)):
            raise FileNotFoundError(
                f"Camera calibration file not found: {camera_calib_file}"
            )

        self.get_logger().info(f"Loading calibration: {camera_calib_file}")

        with open(camera_calib_file, "r") as f:
            cfg = yaml.safe_load(f)

        cam = cfg["camera_mount"]  # expected entry

        xyz = cam["translation"]

        # Allow quaternion or RPY
        if "rotation" in cam:
            q = cam["rotation"]   # [qx, qy, qz, qw]
        else:
            rpy = cam["rotation_rpy"]  # [roll, pitch, yaw]
            q = R.from_euler("xyz", rpy).as_quat()

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "world"
        t.child_frame_id = camera_mount_frame

        t.transform.translation.x = xyz[0]
        t.transform.translation.y = xyz[1]
        t.transform.translation.z = xyz[2]

        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.static_broadcaster.sendTransform(t)

        self.get_logger().info(
            f"Published static TF world --> {camera_mount_frame}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = StaticCameraTFPublisher()
    rclpy.spin(node)
    rclpy.shutdown()
