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

"""ROS 2 node to perform a pick-and-place operation using the UR3e HandE robot.

MoveIt operations are handled via PyMoveIt2. The node subscribes to a MarkerArray topic to also populate the planning scene with a segmented plane for placing objects.

Author: Clinton Enwerem
Developed for the course ENEE467: Robotics Projects Laboratory, Fall 2025, University of Maryland, College Park, MD.
"""

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

import roboticstoolbox as rtb
import spatialmath as sm

from visualization_msgs.msg import Marker, MarkerArray

# Simple “home” joint config 
home_q = [0.0, -1.5707, 0.0, -1.5707, 0.0, 0.0]

class PickAndPlace(Node):
    def __init__(self):
        super().__init__("pnp_demo")
        self.rtb_model = rtb.models.UR3()
        self.start_time = time.time()

        # Subscriber to plane markers from perception
        self.pcd_plane_sub = self.create_subscription(
            MarkerArray,
            "plane_marker",
            self.scene_cb,
            10
        )

        self.objects = {}

        # variables for metrics
        self.plan_times = []
        self.exec_times = []
        self.planning_success = None

        # Parameters
        self.declare_parameter("synchronous", True)
        self.declare_parameter("obj_pos", "")       # "x y z" 
        self.declare_parameter("z_pregrip", 0.23846336)
        self.declare_parameter("z_offset", 0.08)
        self.declare_parameter("place_pos", "")     
        self.declare_parameter("ee_frame", "tool0")
        self.declare_parameter("print_metrics", False)
        self.declare_parameter("max_vel_scale", 0.15)
        self.declare_parameter("max_acc_scale", 0.15)

        self.synchronous = self.get_parameter("synchronous").get_parameter_value().bool_value
        self.zpregrip = self.get_parameter("z_pregrip").get_parameter_value().double_value
        self.zoffset = self.get_parameter("z_offset").get_parameter_value().double_value
        self.ee_frame = self.get_parameter("ee_frame").get_parameter_value().string_value
        self.print_metrics = self.get_parameter("print_metrics").get_parameter_value().bool_value
        self.max_vel_scale = self.get_parameter("max_vel_scale").get_parameter_value().double_value
        self.max_acc_scale = self.get_parameter("max_acc_scale").get_parameter_value().double_value

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

        self.BRIGHT_BLUE = "\x1b[94m"
        self.RESET = "\x1b[0m"


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

        # Compute joint targets with IK
        # TODO (2a) — Compute IK for each sub-task
        # using self.solve_ik(...) with appropriate seeds ie home_q, self.approach_q, etc.

        self.get_logger().info(f"Computing IK for approach at {self.approach_pos}")
        self.approach_q = None

        self.get_logger().info(f"Computing IK for pre-grip at {self.pre_grip_pos}")
        self.pre_grip_q = None

        self.get_logger().info(f"Computing IK for place at {self.place_pos}")
        self.place_q = None

        # lift reuses the approach config with higher Z
        self.after_pick_q = None
        self.retreat_q = None

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

    def _ik_rtb(self, pos_xyz, q0=None):
        """Compute IK using Robotics Toolbox as fallback"""

        # choose initial seed for IK
        q0_seed = q0 if q0 is not None else self.ur_approach
        se_approach = self.rtb_model.fkine(q0_seed, end=self.ee_frame)

        # rtb fallback using the chosen seed's pose
        T_goal = sm.SE3.Trans(x=pos_xyz[0], y=pos_xyz[1], z=pos_xyz[2]) * sm.SE3.Rt(se_approach.R, [0,0,0])
        q_rtb = self.rtb_model.ikine_LM(
            Tep=T_goal,
            q0=q0_seed,
            ilimit=2000,
            end=self.ee_frame,
            tol=1e-1)
        self.get_logger().info(f"RTB IK error norm for pos_xyz with RTB: {pos_xyz}: {q_rtb.residual:.6f}")
        if q_rtb.success:
            self.get_logger().info(f"\x1b[32mRTB IK success at pos_xyz with RTB: {pos_xyz}, with q: {q_rtb.q.tolist()}\x1b[0m")
            return q_rtb.q.tolist()
        else:
            self.get_logger().debug(f"IK failed for pos_xyz with RTB: {pos_xyz}")
            return None
    
    def compute_place_from_plane(self, obj_pos, plane_marker, margin):
        """
        Compute a place position ON the segmented plane, clamped to its bounds.
        """

        # Extract plane pose
        plane_pos = np.array([
            plane_marker.pose.position.x,
            plane_marker.pose.position.y,
            plane_marker.pose.position.z
        ])

        quat = plane_marker.pose.orientation
        plane_quat = np.array([quat.x, quat.y, quat.z, quat.w])

        # Plane rotation matrix in world frame
        R_plane = R.from_quat(plane_quat).as_matrix()

        # Dimensions of plane's collision box
        dx, dy, dz = plane_marker.scale.x, plane_marker.scale.y, plane_marker.scale.z

        # Transform object into plane local coords
        obj_world = np.array(obj_pos)
        obj_local = R_plane.T @ (obj_world - plane_pos)

        # Clamp XY coordinates inside plane footprint
        half_x = dx / 2.0 - margin
        half_y = dy / 2.0 - margin

        obj_local[0] = np.clip(obj_local[0], -half_x, half_x)
        obj_local[1] = np.clip(obj_local[1], -half_y, half_y)

        # Place Z at the plane surface
        place_local = np.array([obj_local[0], obj_local[1], dz / 2.0 + 0.02])  # 2cm above plane

        # Convert back to world coordinates
        place_world = plane_pos + R_plane @ place_local

        return place_world.tolist()
   
    def scene_cb(self, msg: MarkerArray):
        """
        Callback for adding segmented plane to MoveIt planning scene.
        Expects a MarkerArray containing ONE plane marker published by perception.
        The marker must have:
            - pose (position + orientation)
            - scale.x, scale.y, scale.z describing box dimensions
        """

        if msg is None or len(msg.markers) == 0:
            return

        if "segmented_plane" in self.objects:
            self.moveit2.remove_collision_object(id="segmented_plane")
            del self.objects["segmented_plane"]

        plane_marker = msg.markers[0]   # get first marker
        self.last_plane_marker = plane_marker  # cache for place position computation

        # Extract pose
        pos = [
            plane_marker.pose.position.x,
            plane_marker.pose.position.y,
            plane_marker.pose.position.z,
        ]

        quat = [
            plane_marker.pose.orientation.x,
            plane_marker.pose.orientation.y,
            plane_marker.pose.orientation.z,
            plane_marker.pose.orientation.w,
        ]

        # Extract dimensions
        dims = [
            max(plane_marker.scale.x, 0.001),
            max(plane_marker.scale.y, 0.001),
            max(plane_marker.scale.z, 0.001),
        ]

        obj = {
            "shape": "box",
            "id": "segmented_plane",
            "position": pos,
            "quat_xyzw": quat,
            "dimensions": dims,
        }

        self.add_collision_object(obj)
        self.get_logger().info(
            f"Added plane collision box at {pos} with dims {dims} and quat {quat}"
        )

        # small sleep to ensure MoveIt processes scene updates
        time.sleep(0.05)
    
    # Core helpers
    def add_collision_object(self, obj):
        shape = obj["shape"]
        obj_id = obj["id"]
        pos = obj.get("position", [0, 0, 0])
        quat = obj.get("quat_xyzw", [0, 0, 0, 1])
        dims = obj.get("dimensions", [0.1, 0.1, 0.1])

        if shape == "box":
            self.moveit2.add_collision_box(id=obj_id, position=pos, quat_xyzw=quat, size=dims)
        elif shape == "sphere":
            self.moveit2.add_collision_sphere(id=obj_id, position=pos, radius=dims[0])
        elif shape == "cylinder":
            self.moveit2.add_collision_cylinder(
                id=obj_id, position=pos, quat_xyzw=quat, height=dims[0], radius=dims[1]
            )
        elif shape == "cone":
            self.moveit2.add_collision_cone(
                id=obj_id, position=pos, quat_xyzw=quat, height=dims[0], radius=dims[1]
            )
        else:
            raise ValueError(f"Unknown shape '{shape}'")
        self.objects[obj_id] = obj
        self.get_logger().info(f"Added {shape} '{obj_id}' at {pos}")


    # ik helper using MoveIt2 with warm-start
    def solve_ik(self, pos, seed=None):
        if seed is None:
            seed = self.moveit2.joint_state if self.moveit2.joint_state is not None else home_q

        # MoveIt IK
        q = self.moveit2.compute_ik(pos, self.quat_xyzw, start_joint_state=seed)
        if q is not None:
            return list(q)

        # RTB IK
        q_rtb = self._ik_rtb(pos, q0=seed)
        if q_rtb is not None:
            return q_rtb

        # give up
        return None

    # Miror place position if not provided
    def compute_place_pos(self) -> List[float]:
        return [-self.pre_grip_pos[0], self.pre_grip_pos[1], self.pre_grip_pos[2]]

    # Timed motion with metrics logging
    """
     Perform a timed motion to joint configuration q, logging planning and execution times.
    """
    def timed_motion(self, q, label=""):
        self.get_logger().info(f"[{label}] Planning...")

        t0 = time.time()
        self.moveit2.move_to_configuration(q)
        t1 = time.time()

        plan_dt = t1 - t0
        self.plan_times.append(plan_dt)

        self.get_logger().info(f"[{label}] Plan time: {plan_dt:.4f} s")

        # wait for exec
        t2 = time.time()
        self.moveit2.wait_until_executed()
        t3 = time.time()

        exec_dt = t3 - t2
        self.exec_times.append(exec_dt)
        self.get_logger().info(f"[{label}] Exec time: {exec_dt:.4f} s")

    # Pick-and-place sequence
    def start_pnp_sequence(self):
        self.get_logger().info("Waiting for custom Hand-E gripper action server...")
        self.gripper_action_client.wait_for_server()
        self.get_logger().info("Starting pick-and-place sequence...")

        # TODO (2b): IK FAILURE HANDLING
        # If any of [self.approach_q, self.pre_grip_q, self.place_q] is None:
        #   * Log a self.get_logger().error(...)
        #   * and return immediately
        #   * Do NOT call timed_motion() at all

        # Approach
        self.get_logger().info("Moving to approach_q")
        self.timed_motion(self.approach_q, "APPROACH")

        # Pre-grip
        self.get_logger().info("Moving to pre_grip_q")
        self.timed_motion(self.pre_grip_q, "PRE-GRIP")

        # Close gripper
        self.send_gripper_goal(self.close_goal, self.after_gripper_closed)

    def after_gripper_closed(self):
        # Lift
        self.get_logger().info("Gripper closed, lifting...")
        self.timed_motion(self.after_pick_q, "LIFT")

        # Place
        self.get_logger().info("Moving to place_q")
        self.timed_motion(self.place_q, "PLACE")

        # Open gripper
        self.send_gripper_goal(self.open_goal, self.after_gripper_opened)

    def after_gripper_opened(self):
        self.get_logger().info("Gripper opened, retreating to home...")
        self.timed_motion(self.retreat_q, "RETREAT")

        # Summary
        self.get_logger().info("Pick-and-place sequence complete.")
        if self.planning_success is None:
            self.planning_success = True
        if self.print_metrics:
            total_plan_time = sum(self.plan_times)
            total_exec_time = sum(self.exec_times)
            avg_plan_time = total_plan_time / len(self.plan_times)
            avg_exec_time = total_exec_time / len(self.exec_times)
            total_pipeline = time.time() - self.start_time
            self.get_logger().info(self.BRIGHT_BLUE + "-------------------------------------------------------" + self.RESET)
            self.get_logger().info(self.BRIGHT_BLUE + "Metrics Summary (HARDWARE):" + self.RESET)
            self.get_logger().info(self.BRIGHT_BLUE + f"  1. Planning Success (bool): {int(self.planning_success)}" + self.RESET)
            self.get_logger().info(self.BRIGHT_BLUE + f"  2. Avg. Planning time [s]: {avg_plan_time:.4f} s" + self.RESET)
            self.get_logger().info(self.BRIGHT_BLUE + f"  3. Avg. Execution time [s]: {avg_exec_time:.4f} s" + self.RESET)
            self.get_logger().info(self.BRIGHT_BLUE + f"  4. Total Plan time [s]: {total_plan_time:.4f} s" + self.RESET)
            self.get_logger().info(self.BRIGHT_BLUE + f"  5. Total Execution time [s]: {total_exec_time:.4f} s" + self.RESET)
            self.get_logger().info(self.BRIGHT_BLUE + f"  6. Total (PnP Pipeline) [s]: {total_pipeline:.4f} s" + self.RESET)
            self.get_logger().info(self.BRIGHT_BLUE + "-------------------------------------------------------" + self.RESET)

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
