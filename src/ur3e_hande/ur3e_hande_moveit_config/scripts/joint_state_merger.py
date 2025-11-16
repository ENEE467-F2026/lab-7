#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from collections import defaultdict

class JointStateMerger(Node):
    def __init__(self):
        super().__init__('joint_state_merger')
        self.declare_parameter('source_list', value=['/joint_states', '/hande/joint_states'])
        self.declare_parameter('publish_topic', value='/combined_joint_states')
        self.declare_parameter('rate', value=100.0)

        source_list = self.get_parameter('source_list').get_parameter_value().string_array_value
        publish_topic = self.get_parameter('publish_topic').get_parameter_value().string_value
        rate = self.get_parameter('rate').get_parameter_value().double_value

        self.joint_states = {}
        self.subs = [self.create_subscription(JointState, topic, self.make_callback(topic), 10) for topic in source_list]
        self.pub = self.create_publisher(JointState, publish_topic, 10)
        self.timer = self.create_timer(1.0 / rate, self.publish_merged)

    def make_callback(self, topic_name):
        def callback(msg):
            self.joint_states[topic_name] = msg
        return callback

    def publish_merged(self):
        combined = JointState()
        combined.header.stamp = self.get_clock().now().to_msg()
        for msg in self.joint_states.values():
            combined.name.extend(msg.name)
            combined.position.extend(msg.position)
            combined.velocity.extend(msg.velocity)
            combined.effort.extend(msg.effort)
        self.pub.publish(combined)

def main(args=None):
    rclpy.init(args=args)
    node = JointStateMerger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()