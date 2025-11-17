#!/usr/bin/env python3

import rclpy
import time
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

import numpy as np
from scipy.spatial.transform import Rotation as R
from typing import List, Optional

from gripper_action.action import GripperAction
from pymoveit2 import MoveIt2
from pymoveit2.robots import ur

# Simple “home” joint config 
home_q = [0.0, -1.5707, 0.0, -1.5707, 0.0, 0.0]


class PickAndPlace(Node):
    def __init__(self):
        super().__init__("pnp_demo")

        # Parameters
        self.declare_parameter("synchronous", True)
        self.declare_parameter("obj_pos", "0.33931674 0.3942382 0.2380788")       # "x y z" 
        self.declare_parameter("z_pregrip", 0.23846336)
        self.declare_parameter("z_offset", 0.06)
        self.declare_parameter("place_pos", "")     

        self.synchronous = self.get_parameter("synchronous").get_parameter_value().bool_value
        self.zpregrip = self.get_parameter("z_pregrip").get_parameter_value().double_value
        self.zoffset = self.get_parameter("z_offset").get_parameter_value().double_value


        self.declare_parameter("ur_approach",
        [-1.8605, -2.99056, 0.675617, -2.00445, 1.61112, -0.00712473])

        self.declare_parameter("ur_approach_cube",
        [-1.86057, -2.99056, 0.475617, -2.00445, 1.61112, -0.00712473])

        self.declare_parameter("ur_pregrip",
        [-1.83906, -3.10465, 0.656839, -2.02065, 1.61113, -0.00707275])

        self.declare_parameter("ur_place",
        [-0.626256, -3.11027, 0.681228, -2.06301, 1.47058, -0.00735981])

        self.ur_approach      = list(self.get_parameter("ur_approach").value)
        self.ur_approach_cube = list(self.get_parameter("ur_approach_cube").value)
        self.ur_pregrip       = list(self.get_parameter("ur_pregrip").value)
        self.ur_place         = list(self.get_parameter("ur_place").value)


        obj_pos_str = self.get_parameter("obj_pos").get_parameter_value().string_value
        self.obj_pos = [float(x) for x in obj_pos_str.split()] if obj_pos_str else None

        if not self.obj_pos or len(self.obj_pos) != 3:
            self.get_logger().error("Invalid or missing 'obj_pos' parameter. Exiting...")
            rclpy.shutdown()
            return

        # Cartesian waypoints from object pose
        self.approach_pos = [
            self.obj_pos[0],
            self.obj_pos[1],
            self.zpregrip + self.zoffset,
        ]
        self.pre_grip_pos = [
            self.obj_pos[0],
            self.obj_pos[1],
            self.zpregrip,
        ]

        place_pos_str = self.get_parameter("place_pos").get_parameter_value().string_value
        self.place_pos = (
            [float(x) for x in place_pos_str.split()]
            if place_pos_str
            else self.compute_place_pos()
        )

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
            callback_group=self.callback_group,
        )
        self.moveit2.planner_id = "RRTConnectkConfigDefault"
        self.moveit2.max_velocity = 0.2
        self.moveit2.max_acceleration = 0.2

        self.quat_xyzw = R.from_matrix(np.eye(3)).as_quat().tolist()

        # Compute joint targets with MoveIt IK
        self.get_logger().info(f"Computing IK for approach at {self.approach_pos}")
        self.approach_q = self._ik(self.approach_pos, seed=home_q)
        self.get_logger().info(f"Computing IK for pre-grip at {self.pre_grip_pos}")
        self.pre_grip_q = self._ik(self.pre_grip_pos, seed=self.approach_q)

        self.get_logger().info(f"Computing IK for place at {self.place_pos}")
        self.place_q = self._ik(self.place_pos, seed=home_q)

        if self.approach_q is None and np.sign(self.obj_pos[0]) < 0:
            self.approach_q =  self.ur_approach
        if self.approach_q is None and np.sign(self.obj_pos[0]) > 0:
            self.approach_q = self.ur_place
        if self.pre_grip_q is None and np.sign(self.obj_pos[0]) < 0:
            self.pre_grip_q =  self.ur_approach_cube
        if self.pre_grip_q is None and np.sign(self.obj_pos[0]) > 0:
            self.pre_grip_q =  self.ur_place
        if self.place_q is None and np.sign(self.obj_pos[0]) > 0:
            self.place_q = self.ur_pregrip
        if self.place_q is None and np.sign(self.obj_pos[0]) < 0:
            self.place_q = self.ur_place

        # lift reuses the approach config with higher Z
        self.after_pick_q = self.approach_q
        self.retreat_q = home_q

        # Gripper goals 
        self.p_grip_close = 255
        self.p_grip_open = 1
        self.f_grip = 1
        self.u_grip = 1
        self.close_goal = [self.p_grip_close, self.u_grip, self.f_grip]
        self.open_goal = [self.p_grip_open, self.u_grip, self.f_grip]

        self.gripper_action_client = ActionClient(
            self,
            GripperAction,
            "robotiq_grip_action",
            callback_group=self.callback_group,
        )

        # Start sequence
        self.start_pnp_sequence()

    # ik helper using MoveIt2 with warm-start
    def _ik(self, position: List[float], seed: Optional[List[float]] = None):
        """
        Compute IK using MoveIt2, optionally warm-starting from a seed configuration.
        """
        if seed is not None:
            q = self.moveit2.compute_ik(position, self.quat_xyzw, start_joint_state=seed)
        else:
            q = self.moveit2.compute_ik(position, self.quat_xyzw)

        if q is None:
            self.get_logger().warn(f"IK failed for position: {position}")
            return None

        q_list = list(q)
        self.get_logger().info(f"IK success for {position} --> {np.round(q_list, 4).tolist()}")
        return q_list

    # Miror place position if not provided
    def compute_place_pos(self) -> List[float]:
        return [-self.pre_grip_pos[0], self.pre_grip_pos[1], self.pre_grip_pos[2]]

    # Pick-and-place sequence
    def start_pnp_sequence(self):
        self.get_logger().info("Waiting for custom Hand-E gripper action server...")
        self.gripper_action_client.wait_for_server()
        self.get_logger().info("Starting pick-and-place sequence...")

        # Approach
        self.get_logger().info("Moving to approach_q")
        self.moveit2.move_to_configuration(self.approach_q)
        self.moveit2.wait_until_executed()

        # Pre-grip
        self.get_logger().info("Moving to pre_grip_q")
        self.moveit2.move_to_configuration(self.pre_grip_q)
        self.moveit2.wait_until_executed()

        # Close gripper
        self.send_gripper_goal(self.close_goal, self.after_gripper_closed)

    def after_gripper_closed(self):
        # Lift
        self.get_logger().info("Gripper closed, lifting...")
        self.moveit2.move_to_configuration(self.after_pick_q)
        self.moveit2.wait_until_executed()

        # Place
        self.get_logger().info("Moving to place_q")
        self.moveit2.move_to_configuration(self.place_q)
        self.moveit2.wait_until_executed()

        # Open gripper
        self.send_gripper_goal(self.open_goal, self.after_gripper_opened)

    def after_gripper_opened(self):
        self.get_logger().info("Gripper opened, retreating to home...")
        self.moveit2.move_to_configuration(self.retreat_q)
        self.moveit2.wait_until_executed()
        self.get_logger().info("Pick-and-place sequence complete.")

    # Gripper action client
    def send_gripper_goal(self, goal, done_callback=None):
        goal_msg = GripperAction.Goal()
        goal_msg.desired_position = goal[0]
        goal_msg.desired_speed = goal[1]
        goal_msg.desired_force = goal[2]

        future = self.gripper_action_client.send_goal_async(goal_msg)
        future.add_done_callback(
            lambda fut: self.goal_response_callback(fut, done_callback)
        )

    def goal_response_callback(self, future, done_callback):
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().warn("Gripper goal rejected")
                return
            self.get_logger().info("Gripper goal accepted, waiting for result...")
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(
                lambda fut: self.get_result_callback(fut, done_callback)
            )
        except Exception as e:
            self.get_logger().error(f"Gripper goal response error: {e}")

    def get_result_callback(self, future, done_callback):
        try:
            result = future.result().result
            self.get_logger().info(f"Gripper action result: {result}")
            if done_callback:
                done_callback()
        except Exception as e:
            self.get_logger().error(f"Error in get_result_callback: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = PickAndPlace()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
