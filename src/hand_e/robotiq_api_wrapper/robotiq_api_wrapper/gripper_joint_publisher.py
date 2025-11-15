#/usr/bin/env python

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from .robotiq_gripper import *
from rclpy.time import Time

def clip_val(analog_val, min_dig_val, max_dig_val):
    # Clamp analog_val to [0, 255]
    analog_val = max(0, min(255, analog_val))
    return min_dig_val + (analog_val / 255) * (max_dig_val - min_dig_val)

class HandEGripperJointPublisher(Node):
    def __init__(self, node_name="gripper_joint_publisher", topic_name="/hande/joint_states", queue_size=10, timer_period=0.5):
      self.node_name = node_name
      super().__init__(self.node_name)
      self.topic_name = topic_name
      self.queue_size = queue_size
      self.timer_period = timer_period

      # Declare params
      self.declare_parameter('robot_ip', value='192.168.77.22')
      self.declare_parameter('hande_min_distance', value=0.0) # m
      self.declare_parameter('hande_max_distance', value=0.052) # Datasheet says 0.05 m
      self.declare_parameter('hande_joint_name', value='robotiq_hande_left_finger_joint')
      self.declare_parameter('hande_min_speed', value=0.02) # m/s 
      self.declare_parameter('hande_max_speed', value=0.15)
      self.declare_parameter('hande_min_force', value=20) # N
      self.declare_parameter('hande_max_force', value=130) 
      self.declare_parameter('hande_max_position_outer', value=0.062) # useful for grips with the exterior of the gripper; ObjectStatus = 1
      self.declare_parameter('hande_weight', value=0.107) # kg
      self.declare_parameter('hande_speed_scale', value=0.7)
      self.declare_parameter('hande_force_scale', value=0.7)

      ip = self.get_parameter('robot_ip').value
      min_distance = self.get_parameter('hande_min_distance').value
      max_distance = self.get_parameter('hande_max_distance').value

      self.gripper_joint_publisher_ = self.create_publisher(JointState, self.topic_name, self.queue_size)
      self.timer = self.create_timer(self.timer_period, self.publish_gripper_state)

      self.gripper = RobotiqGripper() # create gripper object using Python API

      # Connect to the gripper via the RS485 interface
      def swap_state(state):
        if not state:
          return 'INACTIVE'
        return 'ACTIVE'
      self.get_logger().info("Connecting to the gripper.....")
      self.get_logger().info(f"Gripper is currently {swap_state(self.gripper.is_active)}.")
      self.gripper.connect(ip, 63352)
      if not self.gripper.is_active:
        # Try to activate gripper
        self.gripper.activate(auto_calibrate=False) # gripper has already been calibrated on the pendant
        self.get_logger().info('Activating Gripper')
        if self.gripper.is_active:
          self.get_logger().info('Gripper successfully activated')

      #  min and max positions will be null without calibration, hence we hardcode the values just in case
      self.gripper_min_position = clip_val(self.gripper.get_min_position(), min_distance, max_distance) if self.gripper.get_min_position()  != None else min_distance
      self.gripper_max_position = clip_val(self.gripper.get_max_position(), min_distance, max_distance) if self.gripper.get_max_position()  != None else max_distance

    def publish_gripper_state(self):
      gripper_joint_state = JointState()
      hande_gripper = self.gripper
      analog_gripper_position = hande_gripper.get_current_position()
      analog_gripper_speed = hande_gripper.get_current_speed()
      analog_gripper_force = hande_gripper.get_current_force()
      min_speed = self.get_parameter("hande_min_speed").value
      max_speed = self.get_parameter("hande_max_speed").value
      min_force = self.get_parameter("hande_min_force").value
      max_force = self.get_parameter("hande_max_force").value
      speed_scale = self.get_parameter("hande_speed_scale").value
      force_scale = self.get_parameter("hande_force_scale").value

      self.gripper_position = clip_val(analog_gripper_position, self.gripper_min_position, self.gripper_max_position)
      self.gripper_speed = min(speed_scale * max_speed, clip_val(analog_gripper_speed, min_speed, max_speed))
      self.gripper_force = min(force_scale * max_force, clip_val(analog_gripper_force, min_force, max_force))

      gripper_joint_state.header.stamp = self.get_clock().now().to_msg()

      left_joint = self.get_parameter('hande_joint_name').value
      right_joint = "robotiq_hande_right_finger_joint"  

      gripper_joint_state.name = [left_joint, right_joint]
      gripper_joint_state.position = [
          self.gripper_position,
          -self.gripper_position  
      ]
      gripper_joint_state.velocity = [
          self.gripper_speed,
          -self.gripper_speed
      ]
      gripper_joint_state.effort = [
          self.gripper_force,
          self.gripper_force  
      ]

      self.gripper_joint_publisher_.publish(gripper_joint_state)

      # self.get_logger().info(f"The gripper's current position is: {gripper_joint_state.position[0]} or around {round(100*gripper_joint_state.position[0]/self.gripper_max_position)}% of its full width")  

def main(args=None):
  rclpy.init(args=args)
  gripper_joint_publisher = HandEGripperJointPublisher()
  rclpy.spin(gripper_joint_publisher)
  gripper_joint_publisher.destroy_node()
  rclpy.shutdown()

if __name__ == "__main__":
  main()
