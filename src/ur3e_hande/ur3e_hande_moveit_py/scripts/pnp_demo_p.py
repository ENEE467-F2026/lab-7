#!/usr/bin/env python3
import os, sys
import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.action import ActionClient
from gripper_action.action import GripperAction
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation as R
import roboticstoolbox as rtb
import spatialmath as spmath
from rclpy.callback_groups import ReentrantCallbackGroup
from ament_index_python import get_package_share_directory

from pymoveit2 import MoveIt2, MoveIt2State
from pymoveit2.robots import ur 

# home
home_q = [0.0, -1.5707, 0.0, -1.5707, 0.0, 0.0]

class AsyncPickPlace(Node):
    def __init__(self):
        super().__init__('pnp_demo')

        self.declare_parameter('p_des_open', 1)
        self.declare_parameter('p_des_close', 255)
        self.declare_parameter('u_des', 10)
        self.declare_parameter('f_des', 10)
        self.declare_parameter("z_pregrip", 0.23846336)
        self.declare_parameter("z_offset", 0.06)

        self.p_close = self.get_parameter('p_des_close').value
        self.p_open = self.get_parameter('p_des_open').value
        self.u = self.get_parameter('u_des').value
        self.f = self.get_parameter('f_des').value
        self.zpregrip = self.get_parameter("z_pregrip").get_parameter_value().double_value
        self.zoffset = self.get_parameter("z_offset").get_parameter_value().double_value

        # callback group for allowing callback execution of callbacks in parallel
        callback_group = ReentrantCallbackGroup()

        self.callback_group = callback_group

        # rtb
        self.ur_ = rtb.models.UR3()
        self.ets = self.ur_.ets()

        self.robot = ur
        self.joint_names = self.robot.joint_names()
        self.group_name = self.robot.MOVE_GROUP_ARM
        self.end_effector_name = self.robot.end_effector_name()
        self.callback_group = callback_group
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=self.joint_names,
            base_link_name=self.robot.base_link_name(),
            end_effector_name=self.end_effector_name,
            group_name=self.group_name,
            callback_group=self.callback_group
        )
        self.moveit2.planner_id = "RRTConnectkConfigDefault"
        self.moveit2.max_velocity = 0.2
        self.moveit2.max_acceleration = 0.2

        self.gripper_client = ActionClient(self, GripperAction, 'robotiq_grip_action')
        
        # object pose subber
        self.pose_received = False
        self.create_subscription(PoseStamped, '/object_pose', self.pose_cb, 10)

        # initialize place_q and self.lift_q
        self.place_q = []
        self.lift_q = []

    def pose_cb(self, msg: PoseStamped):
        if self.pose_received:
            return
        self.pose_received = True
        self.get_logger().info("Object pose received, starting pick-and-place...")

        obj = msg.pose.position
        pre = [obj.x, obj.y, self.zpregrip + self.zoffset]
        target = [obj.x, obj.y, self.zoffset]
        place = [-obj.x, obj.y, self.zoffset]  # reflect around y

        # identity rotation 
        quat_xyzw = R.from_matrix(np.eye(3)).as_quat()

        # Compute IK for each position
        self.get_logger().info("Computing IK for pre-grip position...")
        self.pre_grip_q = self.moveit2.compute_ik(pre, quat_xyzw)

        self.get_logger().info("Computing IK for target grip position...")
        self.target_q = self.moveit2.compute_ik(target, quat_xyzw)

        self.get_logger().info("Computing IK for place position...")
        self.place_q = self.moveit2.compute_ik(place, quat_xyzw)

        # compute lift position (same as pre)
        self.lift_q = self.pre_grip_q

        # log
        if not self.pre_grip_q or not self.target_q or not self.place_q:
            self.get_logger().error("Failed to compute IK for one or more target poses.")
        else:
            self.get_logger().info("Successfully computed all IK targets.")
        # Plan pick sequence using pre-computed joint configs
        assert self.pre_grip_q is not None, "No IK solution for pre-grip"
        self.move_to(self.pre_grip_q, next_cb=lambda: self.move_to(self.target_q, next_cb=self.close_gripper))


    def move_to(self, joint_config, next_cb=None):
        self.get_logger().info(f"Moving to configuration: {joint_config}")
        self.moveit2.move_to_configuration(joint_config)
        if self.moveit2.wait_until_executed():
            if next_cb:
                next_cb()
        else:
            self.get_logger().error("MoveToConfig failed")

    def close_gripper(self):
        self.get_logger().info("Closing gripper...")
        self.send_gripper_goal([self.p_close, self.u, self.f], self.after_close)

    def after_close(self):
        # lift
        self.get_logger().info("Gripper closed, now moving robot to lift position")
        self.moveit2.move_to_configuration(self.lift_q)
        self.moveit2.wait_until_executed()

        # place_q
        self.get_logger().info("Moving to place position")
        self.moveit2.move_to_configuration(self.place_q)
        self.moveit2.wait_until_executed()
        self.open_gripper()

    def open_gripper(self):
        self.get_logger().info("Opening gripper...")
        self.send_gripper_goal([self.p_open, self.u, self.f], self.after_open)

    def after_open(self):
        self.get_logger().info("Retreating to home pose")
        self.moveit2.move_to_configuration(home_q)
        self.moveit2.wait_until_executed()
        self.get_logger().info("Pick-and-place complete!")

    def send_gripper_goal(self, goal, done_cb):
        goal_msg = GripperAction.Goal()
        goal_msg.desired_position = goal[0]
        goal_msg.desired_speed = goal[1]
        goal_msg.desired_force = goal[2]
        fut = self.gripper_client.send_goal_async(goal_msg)
        fut.add_done_callback(lambda f: self._on_goal_response(f, done_cb))

    def _on_goal_response(self, future, next_cb):
        gh = future.result()
        if not gh.accepted:
            self.get_logger().warn("Gripper goal rejected")
            return
        self.get_logger().info("Gripper goal accepted")
        rf = gh.get_result_async()
        rf.add_done_callback(lambda f: self._on_result(f, next_cb))

    def _on_result(self, future, next_cb):
        res = future.result().result
        self.get_logger().info(f"Gripper finished: {res}")
        if next_cb:
            next_cb()

def main(args=None):
    rclpy.init(args=args)
    node = AsyncPickPlace()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
