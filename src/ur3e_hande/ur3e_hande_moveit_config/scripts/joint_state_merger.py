#!/usr/bin/env python3

"""
ROS 2 node to merge multiple JointState messages into a single JointState message.
Useful for combining joint states from different sources (e.g., robot arm and gripper).

Usage:
    ros2 run ur3e_hande_moveit_config joint_state_merger.py

Author: Clinton Enwerem
Developed for the course ENEE467: Robotics Projects Laboratory, Fall 2025, University of Maryland, College Park, MD.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStateMerger(Node):
    def __init__(self):
        super().__init__('joint_state_merger')

        # Parameters
        self.declare_parameter('source_list', ['/joint_states', '/hande/joint_states'])
        self.declare_parameter('publish_topic', '/combined_joint_states')
        self.declare_parameter('rate', 100.0)

        self.sources = self.get_parameter('source_list').get_parameter_value().string_array_value
        self.output_topic = self.get_parameter('publish_topic').value
        self.rate = self.get_parameter('rate').value

        self.buffers = {src: None for src in self.sources}

        # Subscriptions
        for src in self.sources:
            self.create_subscription(JointState, src, self._make_cb(src), 10)

        # Publisher
        self.pub = self.create_publisher(JointState, self.output_topic, 10)

        # Timer
        self.timer = self.create_timer(1.0 / self.rate, self._publish)

        # self.get_logger().info(f"Merging: {self.sources} --> {self.output_topic}")

    def _make_cb(self, topic):
        def cb(msg):
            self.buffers[topic] = msg
        return cb

    def _publish(self):
        if any(v is None for v in self.buffers.values()):
            return

        merged = JointState()
        merged.header.stamp = self.get_clock().now().to_msg()

        # ensure consistent ordering of joints
        for topic in sorted(self.buffers.keys()):
            msg = self.buffers[topic]

            merged.name.extend(msg.name)
            merged.position.extend(msg.position)
            if len(msg.velocity) == len(msg.name):
                merged.velocity.extend(msg.velocity)
            else:
                merged.velocity.extend([0.0] * len(msg.name))

            if len(msg.effort) == len(msg.name):
                merged.effort.extend(msg.effort)
            else:
                merged.effort.extend([0.0] * len(msg.name))

        self.pub.publish(merged)


def main(args=None):
    rclpy.init(args=args)
    node = JointStateMerger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
