#!/usr/bin/env python3

# Lab 7: Hardware-Based 3D Perception, Motion Planning, and Control for the UR3e Robot
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
Pick-and-Place Node for the UR3e-Hand-E robot in simulation.

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
import roboticstoolbox as rtb
import spatialmath as sm

from pymoveit2 import MoveIt2
from pymoveit2.robots import ur3e_hande as robot
from control_msgs.action import ParallelGripperCommand
from ur3e_hande_planning_interfaces.action import GetTargetObjPose
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker, MarkerArray

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class PickAndPlaceSim(Node):
    def __init__(self):
        super().__init__("pnp_demo_sim")
        # initialize state early to avoid callback exceptions
        self.objects = {}
        self.last_plane_marker = None
        

        self.rtb_model = rtb.models.UR3()
        self.cb_group = ReentrantCallbackGroup()

        # variables for metrics
        self.start_time = time.time()
        self.plan_times = []
        self.exec_times = []
        self.planning_success = None

        # qos
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        # params
        self.declare_parameter("obj_pos", [])
        self.declare_parameter("target_obj_bounds", [0.3, 0.5, -0.2, 0.2])
        self.declare_parameter("target_height", 0.13)
        self.declare_parameter("target_position", [0.29, 0.51, 0.076])
        self.declare_parameter("pregrasp_z", 0.28)
        self.declare_parameter("lift_z_offset", 0.12)
        self.declare_parameter("place_offset_y", 0.0)
        self.declare_parameter("ee_frame", "tool0")
        self.declare_parameter("grip_exec_delay", 0.7) # s
        self.declare_parameter("safe_lift_z", 0.60)   # 60 cm above table; higher due to simulation artifacts
        self.declare_parameter("print_metrics", False)
        self.declare_parameter("max_vel_scale", 0.25)
        self.declare_parameter("max_acc_scale", 0.25)
        self.declare_parameter("goal_pos_tol", 0.003)
        self.declare_parameter("goal_ori_tol", 0.01)
        self.declare_parameter("place_margin", 0.0)  # m
        self.declare_parameter("plane_offset_z", 0.002)  # m
        self.declare_parameter("table_dims", [0.822, 1.092, 0.755])  # x,y,z
        self.declare_parameter("print_extra_metrics", False)

        self.pregrasp_z = self.get_parameter("pregrasp_z").value
        self.lift_z_offset = self.get_parameter("lift_z_offset").value
        self.place_offset_y = self.get_parameter("place_offset_y").value
        self.target_height = self.get_parameter("target_height").value
        self.target_position = self.get_parameter("target_position").value
        self.ee_frame = self.get_parameter("ee_frame").get_parameter_value().string_value
        self.grip_exec_delay = self.get_parameter("grip_exec_delay").value
        self.safe_lift_z = self.get_parameter("safe_lift_z").value
        self.print_metrics = self.get_parameter("print_metrics").get_parameter_value().bool_value
        self.print_extra_metrics = self.get_parameter("print_extra_metrics").get_parameter_value().bool_value
        self.max_vel_scale = self.get_parameter("max_vel_scale").get_parameter_value().double_value
        self.max_acc_scale = self.get_parameter("max_acc_scale").get_parameter_value().double_value
        self.goal_pos_tol = self.get_parameter("goal_pos_tol").get_parameter_value().double_value
        self.goal_ori_tol = self.get_parameter("goal_ori_tol").get_parameter_value().double_value
        self.place_margin = self.get_parameter("place_margin").get_parameter_value().double_value
        self.plane_offset_z = self.get_parameter("plane_offset_z").get_parameter_value().double_value
        self.table_dims = self.get_parameter("table_dims").get_parameter_value().double_array_value

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

        self.GREEN = "\x1b[32m"
        self.RESET = "\x1b[0m"

        # synchronous execution
        self.declare_parameter("synchronous", True)
        self._synchronous = self.get_parameter("synchronous").get_parameter_value().bool_value

        # gripper client
        self.gripper_client = ActionClient(
            self,
            ParallelGripperCommand,
            "/gripper_action_controller/gripper_cmd",
            callback_group=self.cb_group,
        )

        # resolve object position
        obj_param = list(self.get_parameter("obj_pos").value or [])
        if obj_param and len(obj_param) == 3:
            self.obj_pos = [float(x) for x in obj_param]
            self.get_logger().info(f"Using hard-coded object position: {self.obj_pos}")
        else:
            self.get_logger().info("No obj_pos given ;  querying perception node…")
            self.obj_pos = self.query_object_pose_from_perception()

        if self.obj_pos is None:
            self.get_logger().error("No object position available. Exiting.")
            rclpy.shutdown()
            return

        # derived waypoints
        self.lift_z = max(self.safe_lift_z, self.obj_pos[2] + self.lift_z_offset)
        self.get_logger().info(f"Computed lift_z: {self.lift_z:.3f} m") # always lift to at least safe_lift_z

        x_obj, y_obj, z_obj = self.obj_pos

        # Mirror the object across the robot base
        x_place = -x_obj * 1.2

        # clamp to a safe zone
        TABLE_X, TABLE_Y, SAFE_Z = self.table_dims[0], self.table_dims[1], self.lift_z
        x_place = float(np.clip(x_place, -TABLE_X    / 2, TABLE_X    / 2))
        y_place = float(np.clip(y_obj, -TABLE_Y / 2, TABLE_Y / 2))
        self.declare_parameter("place_pos_arg", [x_place, y_place, SAFE_Z])
        self.place_pos_arg = self.get_parameter("place_pos_arg").get_parameter_value().double_array_value

        self.get_logger().info("\x1b[34m" + f"Manual place position: {self.place_pos_arg}" + "\x1b[0m")
        self.approach_z = max(self.obj_pos[2] + self.lift_z_offset, self.pregrasp_z + 0.03)

        self.approach_pos = [self.obj_pos[0], self.obj_pos[1], self.approach_z]
        self.get_logger().info(f"Approach position: {self.approach_pos}")

        self.grasp_pos = [self.obj_pos[0], self.obj_pos[1], self.pregrasp_z]
        self.get_logger().info(f"Grasp position: {self.grasp_pos}")

        self.lift_pos = [self.obj_pos[0], self.obj_pos[1], self.lift_z]
        self.place_pos = None

        # MoveIt2 Interface
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=robot.joint_names(),
            base_link_name=robot.base_link_name(),
            end_effector_name=robot.end_effector_name(),
            group_name=robot.MOVE_GROUP_ARM,
            callback_group=self.cb_group,
        )
        self.moveit2.planner_id = "RRTConnectkConfigDefault"
        self.moveit2.max_velocity = self.max_vel_scale
        self.moveit2.max_acceleration = self.max_acc_scale
        self.moveit2.goal_position_tolerance = self.goal_pos_tol
        self.moveit2.goal_orientation_tolerance = self.goal_ori_tol

        # subscribe after MoveIt2 and state exist so the first plane message is handled
        self.pcd_plane_sub = self.create_subscription(
            MarkerArray,
            "plane_marker",
            self.scene_cb,
            qos,
        )
 
        # begin sequence
        self._start_timer = self.create_timer(1.5, self._start_move, callback_group=self.cb_group)

    def compute_place_pos(self, pre_grip_pos: list) -> list[float]:
        return [-pre_grip_pos[0], pre_grip_pos[1], pre_grip_pos[2]]
    
    def compute_place_from_plane(self, obj_pos, plane_marker, margin):
        """
        Compute a safe, graceful placement position ON the segmented plane.
        - Projects object onto the plane
        - Applies a small in-plane offset
        - Clamps inside plane bounds
        - Places slightly above plane surface
        """

        # Extract plane geometry
        plane_pos = np.array([
            plane_marker.pose.position.x,
            plane_marker.pose.position.y,
            plane_marker.pose.position.z
        ])

        quat = plane_marker.pose.orientation
        R_plane = R.from_quat([quat.x, quat.y, quat.z, quat.w]).as_matrix()

        x_axis = R_plane[:, 0]
        y_axis = R_plane[:, 1]
        z_axis = R_plane[:, 2]  # plane normal

        dx, dy, dz = plane_marker.scale.x, plane_marker.scale.y, plane_marker.scale.z
        half_x = dx / 2.0 - margin
        half_y = dy / 2.0 - margin

        # Project object onto plane
        obj_world = np.array(obj_pos)
        height = np.dot(obj_world - plane_pos, z_axis)
        obj_on_plane = obj_world - height * z_axis

        # Initial place candidate:
        # Move +12 cm along plane-x
        candidate = obj_on_plane + self.lift_z_offset * x_axis

        # convert place candidate to plane coordinates
        local = np.array([
            np.dot(candidate - plane_pos, x_axis),
            np.dot(candidate - plane_pos, y_axis)
        ])

        # clamp inside plane bounds
        local[0] = np.clip(local[0], -half_x, half_x)
        local[1] = np.clip(local[1], -half_y, half_y)

        # Convert back to world coords
        place_world = (
            plane_pos
            + local[0] * x_axis
            + local[1] * y_axis
            + (0.02 * z_axis)    # 2 cm above plane surface
        )

        return place_world.tolist()

    #  helper methods
    def query_object_pose_from_perception(self):
        """Request an object pose from perception via GetTargetObjPose"""
        self.pose_client = ActionClient(self, GetTargetObjPose, "get_target_obj_pose")
        self.pose_client.wait_for_server()

        goal_msg = GetTargetObjPose.Goal()
        goal_msg.target_position.x = self.target_position[0]
        goal_msg.target_position.y = self.target_position[1]
        goal_msg.target_position.z = self.target_position[2]
        goal_msg.target_height = self.target_height

        self.get_logger().info(f"Requesting nearest object to {self.target_position}…")

        send_future = self.pose_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Pose goal rejected.")
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

    # IK + planning utils
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
        self.get_logger().info(f"RTB IK error norm for pos_xyz: {pos_xyz}: {q_rtb.residual:.6f}")
        if q_rtb.success:
            self.get_logger().info(f"\x1b[32mRTB IK success at pos_xyz: {pos_xyz}, with q: {q_rtb.q.tolist()}\x1b[0m")
            return q_rtb.q.tolist()
        else:
            self.get_logger().debug(f"IK failed for pos_xyz: {pos_xyz}")
            return None

    def _plan_then_execute(self, q, label):
        """Plan a trajectory; only execute if valid"""
        if q is None:
            self.get_logger().warn(f"[{label}] No IK solution ;  skipping.")
            return False
        synchronous = self.get_parameter("synchronous").get_parameter_value().bool_value

        t0 = time.time()
        try:
            traj = self.moveit2.move_to_configuration(q)
        except Exception as e:
            self.get_logger().warn(f"[{label}] Planning/execution threw: {e}")
            return False
        t1 = time.time()
        plan_dt = t1 - t0
        self.plan_times.append(plan_dt)
        self.get_logger().info(f"[{label}] Plan time: {plan_dt:.4f} s")
        
        if traj is not None and not getattr(traj, "joint_trajectory", None):
            self.get_logger().warn(f"[{label}] Planning returned an unexpected result.")

        if synchronous:
            # Wait until execution finishes before continuing 
            try:
                t2 = time.time()
                self.get_logger().info(f"[{label}] Waiting until execution finished...")
                self.moveit2.wait_until_executed()
                t3 = time.time()
                exec_dt = t3 - t2
                self.exec_times.append(exec_dt)
                self.get_logger().info(f"[{label}] Exec time: {exec_dt:.4f} s")

            except Exception as e:
                self.get_logger().warn(f"[{label}] Waiting for execution failed: {e}")
                return False

        return True
        
    def scene_cb(self, msg: MarkerArray):
        """
        Callback for adding segmented plane to MoveIt planning scene.
        Expects a MarkerArray containing ONE plane marker published by perception.
        """
        if msg is None or len(msg.markers) == 0:
            return

        plane_marker = msg.markers[0]
        # cache the latest plane marker immediately
        self.last_plane_marker = plane_marker
        self.get_logger().info("Cached plane marker for place position computation.")

        # scene maintenance
        try:
            if "segmented_plane" in self.objects:
                self.moveit2.remove_collision_object(id="segmented_plane")
                del self.objects["segmented_plane"]
        except Exception as e:
            self.get_logger().debug(f"Scene cleanup skipped: {e}")
 
        # Extract pose
        pos = [
            plane_marker.pose.position.x,
            plane_marker.pose.position.y,
            self.plane_offset_z,
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

        # guard if MoveIt2 not yet ready for any reason
        try:
            self.add_collision_object(obj)
        except Exception as e:
            self.get_logger().debug(f"Add collision object skipped: {e}")

        # small sleep to ensure MoveIt processes scene updates
        time.sleep(0.05)

    def _start_move(self):
        """One-shot starter: cancel timer, wait for joint states, then run sequence."""
        try:
            self._start_timer.cancel()
        except Exception:
            pass

        # Wait briefly for joint states to populate
        wait_logged = False
        start = self.get_clock().now()
        rate = self.create_rate(20)
        while rclpy.ok() and self.moveit2.joint_state is None:
            if not wait_logged:
                self.get_logger().info("Waiting for joint states before starting pick-and-place...")
                wait_logged = True

            rate.sleep()
            if (self.get_clock().now() - start).nanoseconds / 1e9 > 3.0:
                break

        # update cached synchronous flag
        self._synchronous = self.get_parameter("synchronous").get_parameter_value().bool_value
        try:
            self.start_sequence()
        except Exception as e:
            self.get_logger().error(f"Start sequence failed: {e}")

        # prevent re-running
        self._start_move = lambda: None

    # main sequence 
    def start_sequence(self):
        # run the main sequence
        # Wait for plane marker so we can compute place_pos
        if self.place_pos is None:
            self.get_logger().info("Waiting for plane marker to compute place position…")

            # wait for plane marker to arrive
            timeout = 10.0  # seconds
            start_t = time.time()
            while self.last_plane_marker is None and (time.time() - start_t) < timeout:
                # allow callbacks to fire
                rclpy.spin_once(self, timeout_sec=0.1)

            if self.last_plane_marker is not None:
                # Compute place pose ON the segmented plane
                try:
                    self.place_pos = self.compute_place_from_plane(
                        self.obj_pos,
                        self.last_plane_marker,
                        self.place_margin,
                    )
                    if self.place_pos:
                        # adjust sign and z
                        self.place_pos[0] = np.sign(self.place_pos_arg[0]) * min(abs(self.place_pos[0]), abs(self.place_pos_arg[0]))
                        self.place_pos[2] = self.place_pos_arg[2]
                    self.get_logger().info(
                        f"\x1b[34mComputed place_pos in start_sequence: {self.place_pos}\x1b[0m"
                    )
                except Exception as e:
                    self.get_logger().error(f"Failed computing place_pos: {e}")
                    self.get_logger().warn(
                        "Falling back to manual place_pos_arg due to plane computation error."
                    )
                    self.place_pos = list(self.place_pos_arg)
            else:
                # No plane ever arrived during timeout
                self.get_logger().warn(
                    "Plane marker did not arrive in time. Using fallback manual place_pos_arg."
                )
                self.place_pos = list(self.place_pos_arg)

        self.get_logger().info("Waiting for gripper action server…")
        self.gripper_client.wait_for_server()
        self.open_gripper()
        time.sleep(self.grip_exec_delay * 4)
        # approach
        q_approach = self._ik_rtb(self.approach_pos)
        if not self._plan_then_execute(q_approach, "Approach"):
            self.get_logger().error("Aborting: cannot reach approach pose.")
            return
        
        # grasp
        # use cube-specific approach as IK seed for grasp
        q_grasp = self._ik_rtb(self.grasp_pos, q0=self.ur_pregrip)
        if not self._plan_then_execute(q_grasp, "Grasp"):
            self.get_logger().error("Aborting: cannot reach grasp pose.")
            return

        self.close_gripper()
        time.sleep(self.grip_exec_delay)

        # lift
        q_lift = self._ik_rtb(self.lift_pos, q0=q_grasp)
        if not self._plan_then_execute(q_lift, "Lift"):
            self.get_logger().error("Aborting: cannot lift after grasp.")
            return

        # place
        if self.place_pos is None:
            self.get_logger().error("Aborting: no valid place position available.")
            return
        q_place = self._ik_rtb(self.place_pos, q0=self.ur_place)
        if not self._plan_then_execute(q_place, "Place"):
            self.get_logger().error("Aborting: cannot reach place pose.")
            return
        time.sleep(self.grip_exec_delay)
        self.open_gripper()
        # return home
        try:
            home = robot.get_named_group_states("")["ur_home"]
            if not self._plan_then_execute(home, "Home"):
                self.get_logger().warn("Could not return to home pose.")
            else:
                # close the gripper once the arm retreats to home
                self.get_logger().info("Retreated to home; closing gripper.")
                self.close_gripper()
        except Exception as e:
            self.get_logger().warn(f"Home pose retrieval/plan failed: {e}")

        self.get_logger().info("Pick-and-place sequence complete.")
        if self.planning_success is None:
            self.planning_success = True
        if self.print_metrics:
            total_plan_time = sum(self.plan_times)
            total_exec_time = sum(self.exec_times)
            avg_plan_time = total_plan_time / len(self.plan_times)
            avg_exec_time = total_exec_time / len(self.exec_times)
            total_pipeline = time.time() - self.start_time
            if self.print_extra_metrics:
                self.get_logger().info(self.GREEN + "-------------------------------------------------------" + self.RESET)
                self.get_logger().info(self.GREEN + f"---------------------- MAX_VEL_SCALE = {self.max_vel_scale} -----------------------" + self.RESET)
                self.get_logger().info(self.GREEN + "-------------------------------------------------------" + self.RESET)
            self.get_logger().info(self.GREEN + "-------------------------------------------------------" + self.RESET)
            self.get_logger().info(self.GREEN + "Metrics Summary:" + self.RESET)
            self.get_logger().info(self.GREEN + f"  1. Planning Success (bool): {int(self.planning_success)}" + self.RESET)
            self.get_logger().info(self.GREEN + f"  2. Planning time [s]: {avg_plan_time:.4f} s" + self.RESET)
            self.get_logger().info(self.GREEN + f"  3. Execution time [s]: {avg_exec_time:.4f} s" + self.RESET)
            self.get_logger().info(self.GREEN + f"  4. Time (PnP Pipeline) [s]: {total_pipeline:.4f} s" + self.RESET)
            self.get_logger().info(self.GREEN + "-------------------------------------------------------" + self.RESET)
            if self.print_extra_metrics:
                self.get_logger().info(self.GREEN + "-------------------------------------------------------" + self.RESET)
                self.get_logger().info(self.GREEN + f"  5. Total Planning time [s]: {total_plan_time:.4f} s" + self.RESET)
                self.get_logger().info(self.GREEN + f"  6. Total Execution time [s]: {total_exec_time:.4f} s" + self.RESET)
                self.get_logger().info(self.GREEN + f"  7. Total (PnP Pipeline) [s]: {total_pipeline:.4f} s" + self.RESET)
                self.get_logger().info(self.GREEN + "-------------------------------------------------------" + self.RESET)

    # gripper methods
    def send_gripper_goal(self, position: float):
        goal = ParallelGripperCommand.Goal()
        goal.command.position = [position]
        future = self.gripper_client.send_goal_async(goal)
        future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn("Gripper goal rejected.")
            return
        handle.get_result_async().add_done_callback(self._result_cb)

    def _result_cb(self, future):
        result = future.result().result
        self.get_logger().debug(f"Gripper result: {result}")

    def open_gripper(self):
        self.get_logger().info("Opening gripper…")
        self.send_gripper_goal(0.025)

    def close_gripper(self):
        self.get_logger().info("Closing gripper…")
        self.send_gripper_goal(0.001)
    
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