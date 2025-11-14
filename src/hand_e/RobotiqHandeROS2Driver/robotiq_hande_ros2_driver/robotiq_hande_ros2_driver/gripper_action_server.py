#!/usr/bin/env python3

# Modified from the file: hand_e/RobotiqHandeROS2Driver/robotiq_hande_ros2_driver/robotiq_hande_ros2_driver/gripper_node.py

import rclpy 
from rclpy.node import Node
from std_msgs.msg import Int32
from .robotiq_gripper import *
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from gripper_action.action import GripperAction

class HandEGripperActionServer(Node):
    def __init__(self):
        super().__init__('gripper_action_server')

        # parms
        self.declare_parameter('robot_ip', value='192.168.77.22')
        self.declare_parameter('p_diff_thresh', value=5)
        self.declare_parameter('u_diff_thresh', value=4)
        self.declare_parameter('f_diff_thresh', value=5)

        self.ip = self.get_parameter('robot_ip').value
        self.p_diff_thresh = self.get_parameter('p_diff_thresh').value
        self.u_diff_thresh = self.get_parameter('u_diff_thresh').value
        self.f_diff_thresh = self.get_parameter('f_diff_thresh').value

        # initialize the gripper
        self.gripper = RobotiqGripper()
        self.get_logger().info("Connecting to the gripper.....")
        self.gripper.connect(self.ip, 63352)
        self.get_logger().info("Activating the gripper.....")
        self.gripper.activate(auto_calibrate=False)

        # set up action server
        self.gripper_act_server = ActionServer(
                                        self, 
                                        GripperAction, 
                                        'robotiq_grip_action', 
                                        self.execute_action_cb)

        self.get_logger().info("Gripper ready to receive action goal...")
    
    def execute_action_cb(self, goal_h: ServerGoalHandle):
        target_position = goal_h.request.desired_position
        target_speed = goal_h.request.desired_speed
        target_force = goal_h.request.desired_force

        result = GripperAction.Result()
        feedback = GripperAction.Feedback()

        if target_speed > 255 or target_speed <=0:
            self.get_logger().warn('invalid speed value. Valid in range (0,255]')
            feedback.current_position = self.gripper.get_current_position()
            feedback.current_speed = 0
            feedback.current_force = self.gripper.get_current_force()
            goal_h.publish_feedback(feedback)
            result.position_error = 0
            result.speed_error = 0
            result.force_error = 0
            goal_h.abort()
            return result
        if target_force > 255 or target_force <=0:
            self.get_logger().warn('invalid force value. Valid in range (0,255]')
            feedback.current_position = self.gripper.get_current_position()
            feedback.current_speed = 0
            feedback.current_force = self.gripper.get_current_force()
            goal_h.publish_feedback(feedback)
            result.position_error = 0
            result.speed_error = 0
            result.force_error = 0
            goal_h.abort()
            return result
        if target_position > 255 or target_position < 0:
            self.get_logger().warn('invalid position value. Valid in range (0,255]')
            feedback.current_position = self.gripper.get_current_position()
            feedback.current_speed = 0
            feedback.current_force = self.gripper.get_current_force()
            goal_h.publish_feedback(feedback)
            result.position_error = 0
            result.speed_error = 0
            result.force_error = 0
            goal_h.abort()
            return result
        p_diff = abs(target_position - self.gripper.get_current_position())
        u_diff = abs(target_speed - self.gripper.get_current_speed())
        f_diff = abs(target_force - self.gripper.get_current_force())

        
        # self.get_logger().info("moving the gripper. position = {}, speed={}, force={}".format(target_position, target_speed, target_force))
        self.gripper.move_and_wait_for_pos(target_position, target_speed, target_force)
        self.get_logger().info('Successfully reached goal!')
        feedback.current_position = self.gripper.get_current_position()
        feedback.current_force = self.gripper.get_current_force()
        feedback.current_speed = self.gripper.get_current_speed()
        goal_h.publish_feedback(feedback)
        result.position_error = p_diff
        result.speed_error = u_diff
        result.force_error = f_diff
        goal_h.succeed()
        return result

def main(args=None):
    rclpy.init(args=args)
    node = HandEGripperActionServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()