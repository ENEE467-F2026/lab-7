#!/usr/bin/env python3

# Lab 7: Autonomous Manipulation with ROS 2 on the Real UR3e-Hand-E Robot
# Copyright (C) 2025 Clinton Enwerem

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#     http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Pick-and-Place Node for the UR3e-Hand-E robot in Simulation.

Behavior:
    - If 'obj_pos' is passed as a ROS parameter (3 floats), it uses it directly.
    - Otherwise, it uses the GetTargetObjPose action client to request
      a detected object's pose from perception based on provided bounds.

Author: Clinton Enwerem
Developed for the course ENEE467: Robotics Projects Laboratory, Fall 2025, University of Maryland, College Park, MD.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from scipy.spatial.transform import Rotation as R
import numpy as np
import time
import sys

from pymoveit2 import MoveIt2
from pymoveit2.robots import ur3e_hande as robot
from control_msgs.action import ParallelGripperCommand
from ur3e_hande_planning_interfaces.action import GetTargetObjPose


class PickAndPlaceSim(Node):
    def __init__(self):
        super().__init__("pnp_demo_sim")
        self.cb_group = ReentrantCallbackGroup()

        # Parameters
        self.declare_parameter("obj_pos", None)
        self.declare_parameter("target_obj_bounds", [0.3, 0.5, -0.2, 0.2])
        self.declare_parameter("pregrasp_z", 0.20)
        self.declare_parameter("lift_z_offset", 0.08)
        self.declare_parameter("place_offset_y", 0.0)

        self.pregrasp_z = self.get_parameter("pregrasp_z").value
        self.lift_z_offset = self.get_parameter("lift_z_offset").value
        self.place_offset_y = self.get_parameter("place_offset_y").value

        # Gripper Client
        self.gripper_client = ActionClient(
            self,
            ParallelGripperCommand,
            "/gripper_action_controller/gripper_cmd",
            callback_group=self.cb_group,
        )

        # Object Pose Source
        self.obj_pos = None
        obj_param = self.get_parameter("obj_pos").value
        if obj_param is not None and len(obj_param) == 3:
            self.obj_pos = [float(x) for x in obj_param]
            self.get_logger().info(f"Using hardcoded object position: {self.obj_pos}")
        else:
            self.get_logger().info("No obj_pos given — querying perception node...")
            self.obj_pos = self.query_object_pose_from_perception()

        if self.obj_pos is None:
            self.get_logger().error("No object position available. Exiting.")
            rclpy.shutdown()
            return

        # Derived poses
        self.lift_z = self.obj_pos[2] + self.lift_z_offset
        self.approach_pos = [self.obj_pos[0], self.obj_pos[1], self.lift_z]
        self.grasp_pos = [self.obj_pos[0], self.obj_pos[1], self.pregrasp_z]
        self.place_pos = [
            self.obj_pos[0],
            self.obj_pos[1] + self.place_offset_y,
            self.lift_z,
        ]

        # MoveIt2 setup
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

        # Start workflow
        self.start_sequence()

    # object pose query via action client
    def query_object_pose_from_perception(self):
        """Calls the GetTargetObjPose action server with target bounds."""
        bounds = self.get_parameter("target_obj_bounds").value
        self.pose_client = ActionClient(self, GetTargetObjPose, "get_target_obj_pose")
        self.pose_client.wait_for_server()
        goal_msg = GetTargetObjPose.Goal()
        goal_msg.target_obj_bounds = [float(x) for x in bounds]
        self.get_logger().info(f"Requesting object pose for bounds {bounds}...")

        send_future = self.pose_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Pose goal was rejected.")
            return None

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        if result.target_obj_found:
            pose = result.target_obj_pose.pose.position
            obj_pos = [pose.x, pose.y, pose.z]
            self.get_logger().info(f"Received object pose: {obj_pos}")
            return obj_pos
        else:
            self.get_logger().warn("No matching object found.")
            return None

    # pick-and-place sequence
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

    # gripper control
    def send_gripper_goal(self, position: float):
        goal = ParallelGripperCommand.Goal()
        goal.command.position = [position]
        future = self.gripper_client.send_goal_async(goal)
        future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn("Gripper goal rejected")
            return
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
