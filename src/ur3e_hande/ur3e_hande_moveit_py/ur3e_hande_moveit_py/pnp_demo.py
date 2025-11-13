#!/usr/bin/env python3

import rclpy
import time
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

import os, sys
import numpy as np
from ament_index_python import get_package_share_directory
import roboticstoolbox as rtb
from scipy.spatial.transform import Rotation as R
from typing import List

from gripper_action.action import GripperAction
from pymoveit2 import MoveIt2
from pymoveit2.robots import ur
from ur3e_hande_planning_interfaces.action import GetTargetObjPose

# home config
home_q = [0.0, -1.5707, 0.0, -1.5707, 0.0, 0.0]

class PickAndPlace(Node):
    def __init__(self):
        super().__init__('pnp_demo')

        # Parameters
        self.declare_parameter("synchronous", True)
        self.declare_parameter("obj_pos", "")
        # params for perception-based object lookup (used when obj_pos not provided)
        self.declare_parameter("target_position", [0.29, 0.51, 0.07])
        self.declare_parameter("target_height", 0.13)
        self.declare_parameter("z_pregrip", 0.23846336)
        self.declare_parameter("z_offset", 0.06)
        self.declare_parameter("place_pos", "")

        self.synchronous = self.get_parameter("synchronous").get_parameter_value().bool_value
        self.zpregrip = self.get_parameter("z_pregrip").get_parameter_value().double_value
        self.zoffset = self.get_parameter("z_offset").get_parameter_value().double_value
        obj_pos_param = self.get_parameter("obj_pos").get_parameter_value()
        # Try to read as double array; fall back to empty
        try:
            obj_pos_val = obj_pos_param.double_array_value
        except Exception:
            obj_pos_val = []

        if obj_pos_val and len(obj_pos_val) == 3:
            self.obj_pos = [float(x) for x in obj_pos_val]
        else:
            self.get_logger().info("No obj_pos provided; querying perception for object pose...")
            self.obj_pos = self.query_object_pose_from_perception()
            if self.obj_pos is None:
                self.get_logger().error("No object pose available from perception. Exiting.")
                rclpy.shutdown()
                return
        # mirror the z_pregrip param into a pregrasp_z name used elsewhere
        self.pregrasp_z = self.zpregrip
        # lift/placement params may be absent in this file; safely attempt to read
        try:
            self.lift_z = self.obj_pos[2] + self.get_parameter("lift_z_offset").value
        except Exception:
            # default lift offset
            self.lift_z = self.obj_pos[2] + 0.06
        try:
            self.place_offset_y = self.get_parameter("place_offset_y").value
        except Exception:
            self.place_offset_y = 0.0

        # relocation poses
        self.approach_pos = [self.obj_pos[0], self.obj_pos[1], self.zpregrip + self.zoffset]
        self.pre_grip_pos = [self.obj_pos[0], self.obj_pos[1], self.zpregrip]

        place_pos_str = self.get_parameter("place_pos").get_parameter_value().string_value
        self.place_pos = [float(x) for x in place_pos_str.split()] if place_pos_str else self.compute_place_pos()

        # MoveIt setup
        self.robot = ur
        self.joint_names = self.robot.joint_names()
        self.group_name = self.robot.MOVE_GROUP_ARM
        self.end_effector_name = self.robot.end_effector_name()
        self.callback_group = ReentrantCallbackGroup()

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

        # Orientation
        quat_xyzw = R.from_matrix(np.eye(3)).as_quat() # assume goal orientation is immaterial

        # Compute IK
        if self.synchronous:
            self.approach_q = self.moveit2.compute_ik(self.approach_pos, quat_xyzw)
            self.pre_grip_q = self.moveit2.compute_ik(self.pre_grip_pos, quat_xyzw)
            self.place_q = self.moveit2.compute_ik(self.place_pos, quat_xyzw)
        else:
            future_approach = self.moveit2.compute_ik_async(self.approach_pos, quat_xyzw)
            future_place = self.moveit2.compute_ik_async(self.place_pos, quat_xyzw)

            rate = self.create_rate(10)
            # check the future states of the approach and place tasks
            while not future_approach.done() or not future_place.done():
                rate.sleep()

            self.approach_q = self.moveit2.get_compute_ik_result(future_approach)
            self.place_q = self.moveit2.get_compute_ik_result(future_place)
            self.pre_grip_q = self.moveit2.compute_ik(self.pre_grip_pos, quat_xyzw)

        self.after_pick_q = self.approach_q
        self.retreat_q = home_q

        # Gripper setup
        self.p_grip_close = 255
        self.p_grip_open = 1
        self.f_grip = 1
        self.u_grip = 1
        self.close_goal = [self.p_grip_close, self.u_grip, self.f_grip]
        self.open_goal = [self.p_grip_open, self.u_grip, self.f_grip]

        self.gripper_action_client = ActionClient(
            self,
            GripperAction,
            'robotiq_grip_action'
        )

        self.start_pnp_sequence()

    def compute_place_pos(self) -> List[float]:
        return [-self.pre_grip_pos[0], self.pre_grip_pos[1], self.pre_grip_pos[2]]

    def query_object_pose_from_perception(self):
        """Request an object pose from perception via GetTargetObjPose action."""
        client = ActionClient(self, GetTargetObjPose, "get_target_obj_pose")
        # wait for server
        client.wait_for_server()

        goal_msg = GetTargetObjPose.Goal()
        tgt = self.get_parameter("target_position").get_parameter_value().double_array_value
        if not tgt or len(tgt) != 3:
            tgt = [0.29, 0.51, 0.07]
        goal_msg.target_position.x = float(tgt[0])
        goal_msg.target_position.y = float(tgt[1])
        goal_msg.target_position.z = float(tgt[2])
        goal_msg.target_height = float(self.get_parameter("target_height").get_parameter_value().double_value)

        self.get_logger().info(f"Requesting nearest object to {tgt} via perception action...")
        send_future = client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Perception action goal rejected.")
            return None

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        if getattr(result, "target_obj_found", False):
            pose = result.target_obj_pose.pose.position
            obj_pos = [pose.x, pose.y, pose.z]
            self.get_logger().info(f"Perception returned object pose: {obj_pos}")
            return obj_pos
        else:
            self.get_logger().warn("Perception did not find a matching object.")
            return None

    def start_pnp_sequence(self):
        self.get_logger().info("Waiting for gripper action server...")
        self.gripper_action_client.wait_for_server()
        self.get_logger().info("Starting pick and place sequence...")

        self.get_logger().info("Moving to approach_q")
        self.moveit2.move_to_configuration(self.approach_q)
        self.moveit2.wait_until_executed()

        self.get_logger().info("Moving to pre_grip_q")
        self.moveit2.move_to_configuration(self.pre_grip_q)
        self.moveit2.wait_until_executed()

        self.send_gripper_goal(self.close_goal, self.after_gripper_closed)

    def after_gripper_closed(self):
        self.get_logger().info("Gripper closed, lifting...")
        self.moveit2.move_to_configuration(self.after_pick_q)
        self.moveit2.wait_until_executed()

        self.get_logger().info("Moving to place_q")
        self.moveit2.move_to_configuration(self.place_q)
        self.moveit2.wait_until_executed()

        self.send_gripper_goal(self.open_goal, self.after_gripper_opened)

    def after_gripper_opened(self):
        self.get_logger().info("Gripper opened, retreating to home")
        self.moveit2.move_to_configuration(self.retreat_q)
        self.moveit2.wait_until_executed()
        self.get_logger().info("Pick and place sequence complete.")

    def send_gripper_goal(self, goal, done_callback):
        goal_msg = GripperAction.Goal()
        goal_msg.desired_position = goal[0]
        goal_msg.desired_speed = goal[1]
        goal_msg.desired_force = goal[2]

        future = self.gripper_action_client.send_goal_async(goal_msg)
        future.add_done_callback(lambda fut: self.goal_response_callback(fut, done_callback))

    def goal_response_callback(self, future, done_callback):
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().warn('Gripper goal rejected')
                return
            self.get_logger().info('Gripper goal accepted, waiting for result...')
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(lambda fut: self.get_result_callback(fut, done_callback))
        except Exception as e:
            self.get_logger().error(f'Gripper goal response error: {e}')

    def get_result_callback(self, future, done_callback):
        try:
            result = future.result().result
            self.get_logger().info(f'Gripper action result: {result}')
            if done_callback:
                done_callback()
        except Exception as e:
            self.get_logger().error(f'Error in get_result_callback: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = PickAndPlace()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
