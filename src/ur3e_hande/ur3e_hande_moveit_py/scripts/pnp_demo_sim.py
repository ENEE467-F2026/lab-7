#!/usr/bin/env python3
"""
Simulation version of the Pick-and-Place node for the UR3e-HandE robot.

Assumes:
  • Gazebo + ros2_control are running and expose /gripper_action_controller/gripper_cmd
  • MoveIt2 controls the arm group via PyMoveIt2
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from scipy.spatial.transform import Rotation as R
import numpy as np
import time

from pymoveit2 import MoveIt2
from pymoveit2.robots import ur3e_hande as robot
from control_msgs.action import ParallelGripperCommand


class PickAndPlaceSim(Node):
    def __init__(self):
        super().__init__("pnp_demo_sim")
        self.cb_group = ReentrantCallbackGroup()

        # Parameters
        self.declare_parameter("obj_pos", "")
        self.declare_parameter("pregrasp_z", 0.20)
        self.declare_parameter("lift_z_offset", 0.08)
        self.declare_parameter("place_offset_y", 0.0)

        obj_pos_str = self.get_parameter("obj_pos").get_parameter_value().string_value
        if not obj_pos_str:
            self.get_logger().error("Missing obj_pos parameter (x y z). Exiting.")
            rclpy.shutdown()
            return
        self.obj_pos = [float(x) for x in obj_pos_str.split()]
        self.pregrasp_z = self.get_parameter("pregrasp_z").value
        self.lift_z = self.obj_pos[2] + self.get_parameter("lift_z_offset").value
        self.place_offset_y = self.get_parameter("place_offset_y").value

        # Define approach, grasp, and place points
        self.approach_pos = [self.obj_pos[0], self.obj_pos[1], self.lift_z]
        self.grasp_pos = [self.obj_pos[0], self.obj_pos[1], self.pregrasp_z]
        self.place_pos = [
            self.obj_pos[0],
            self.obj_pos[1] + self.place_offset_y,
            self.lift_z,
        ]

        # MoveIt2 interface
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=robot.joint_names(),
            base_link_name=robot.base_link_name(),
            end_effector_name=robot.end_effector_name(),
            group_name=robot.MOVE_GROUP_ARM,
            callback_group=self.cb_group,
        )
        self.moveit2.planner_id = "RRTConnectkConfigDefault"
        self.moveit2.max_velocity = 0.25
        self.moveit2.max_acceleration = 0.25

        # Gripper action client
        self.gripper_client = ActionClient(
            self,
            ParallelGripperCommand,
            "/gripper_action_controller/gripper_cmd",
            callback_group=self.cb_group,
        )

        self.start_sequence()

    # workflow
    def start_sequence(self):
        self.get_logger().info("Waiting for gripper action server...")
        self.gripper_client.wait_for_server()

        quat_xyzw = R.from_matrix(np.eye(3)).as_quat()

        q_approach = self.moveit2.compute_ik(self.approach_pos, quat_xyzw)
        q_grasp = self.moveit2.compute_ik(self.grasp_pos, quat_xyzw)
        q_place = self.moveit2.compute_ik(self.place_pos, quat_xyzw)

        self.get_logger().info("Moving to approach position...")
        self.moveit2.move_to_configuration(q_approach)
        self.moveit2.wait_until_executed()

        self.get_logger().info("Moving to grasp position...")
        self.moveit2.move_to_configuration(q_grasp)
        self.moveit2.wait_until_executed()

        self.close_gripper()
        time.sleep(1.0)

        self.get_logger().info("Lifting object...")
        self.moveit2.move_to_configuration(q_approach)
        self.moveit2.wait_until_executed()

        self.get_logger().info("Moving to place position...")
        self.moveit2.move_to_configuration(q_place)
        self.moveit2.wait_until_executed()

        self.open_gripper()
        time.sleep(1.0)

        self.get_logger().info("Returning to home pose...")
        self.moveit2.move_to_configuration(robot.get_named_group_states("")["ur_home"])
        self.moveit2.wait_until_executed()
        self.get_logger().info("Pick-and-place sequence complete.")

    # hande goal
    def send_gripper_goal(self, position: float):
        goal = ParallelGripperCommand.Goal()
        goal.command.position = position
        future = self.gripper_client.send_goal_async(goal)
        future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn("Gripper goal rejected")
            return
        self.get_logger().debug("Gripper goal accepted")
        handle.get_result_async().add_done_callback(self._result_cb)

    def _result_cb(self, future):
        result = future.result().result
        self.get_logger().debug(f"Gripper result: {result}")

    def open_gripper(self):
        self.get_logger().info("Opening gripper...")
        self.send_gripper_goal(0.025)

    def close_gripper(self):
        self.get_logger().info("Closing gripper...")
        self.send_gripper_goal(0.002)


def main(args=None):
    rclpy.init(args=args)
    node = PickAndPlaceSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()