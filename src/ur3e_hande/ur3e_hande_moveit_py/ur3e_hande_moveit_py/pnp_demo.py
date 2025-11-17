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
Pick-and-Place Node for the real UR3e-Hand-E robot.

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

class PickAndPlace(Node):
    def __init__(self):
        super().__init__("pnp_demo")
        self.rtb_model = rtb.models.UR3()
        self.cb_group = ReentrantCallbackGroup()

        self.pcd_plane_sub = self.create_subscription(
            MarkerArray,
            "plane_marker",
            self.scene_cb,
            10
        )

        self.objects = {}

        # variables for metrics
        self.plan_time = None
        self.exec_time = None
        self.planning_success = None

        # params
        self.declare_parameter("obj_pos", [-0.33931674, 0.3942382, 0.2380788])
        self.declare_parameter("target_obj_bounds", [0.3, 0.5, -0.2, 0.2])
        self.declare_parameter("target_height", 0.13)
        self.declare_parameter("target_position", [0.29, 0.51, 0.10])
        self.declare_parameter("pregrasp_z", 0.11)
        self.declare_parameter("place_z", 0.4)
        self.declare_parameter("lift_z_offset", 0.12)
        self.declare_parameter("place_offset_y", 0.0)
        self.declare_parameter("ee_frame", "tool0")
        self.declare_parameter("grip_exec_delay", 0.9) # s
        self.declare_parameter("safe_lift_z", 0.30)  
        self.declare_parameter("print_metrics", False)
        self.declare_parameter("max_vel_scale", 0.15)
        self.declare_parameter("max_acc_scale", 0.15)
        self.declare_parameter("goal_pos_tol", 0.003)
        self.declare_parameter("goal_ori_tol", 0.01)
        self.declare_parameter("place_margin", 0.1)  # m

        self.pregrasp_z = self.get_parameter("pregrasp_z").value
        self.lift_z_offset = self.get_parameter("lift_z_offset").value
        self.place_offset_y = self.get_parameter("place_offset_y").value
        self.target_height = self.get_parameter("target_height").value
        self.goal_pos_tol = self.get_parameter("goal_pos_tol").get_parameter_value().double_value
        self.goal_ori_tol = self.get_parameter("goal_ori_tol").get_parameter_value().double_value
        self.target_position = self.get_parameter("target_position").value
        self.ee_frame = self.get_parameter("ee_frame").get_parameter_value().string_value
        self.grip_exec_delay = self.get_parameter("grip_exec_delay").value
        self.place_z = self.get_parameter("place_z").value
        self.safe_lift_z = self.get_parameter("safe_lift_z").value
        self.print_metrics = self.get_parameter("print_metrics").get_parameter_value().bool_value
        self.max_vel_scale = self.get_parameter("max_vel_scale").get_parameter_value().double_value
        self.max_acc_scale = self.get_parameter("max_acc_scale").get_parameter_value().double_value
        self.place_margin = self.get_parameter("place_margin").get_parameter_value().double_value

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

        self.declare_parameter("place_pos_arg", [-self.target_position[0]-0.1, 
                                                 self.target_position[1], 
                                                 self.lift_z])
        self.place_pos_arg = self.get_parameter("place_pos_arg").get_parameter_value().double_array_value
        self.approach_z = max(self.obj_pos[2] + self.lift_z_offset, self.pregrasp_z + 0.03)

        self.approach_pos = [self.obj_pos[0], self.obj_pos[1], self.approach_z]
        self.get_logger().info(f"Approach position: {self.approach_pos}")

        self.grasp_pos = [self.obj_pos[0], self.obj_pos[1], self.pregrasp_z]
        self.get_logger().info(f"Grasp position: {self.grasp_pos}")

        self.lift_pos = [self.obj_pos[0], self.obj_pos[1], self.lift_z]

        if hasattr(self, "last_plane_marker"):
            self.place_pos = self.compute_place_from_plane(self.obj_pos, self.last_plane_marker, self.place_margin)
            self.get_logger().info(f"Auto-chosen place position: {self.place_pos}")
        else:
            # fallback 
            self.place_pos = self.compute_place_pos(self.grasp_pos)

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

        # begin sequence
        self._start_timer = self.create_timer(1.5, self._start_move, callback_group=self.cb_group)

    def compute_place_pos(self, pre_grip_pos: list) -> list[float]:
        return [-pre_grip_pos[0], pre_grip_pos[1], pre_grip_pos[2]]
    
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
        if self.plan_time is None:
            self.plan_time = t1 - t0
        
        if traj is not None and not getattr(traj, "joint_trajectory", None):
            self.get_logger().warn(f"[{label}] Planning returned an unexpected result.")

        if synchronous:
            # Wait until execution finishes before continuing 
            try:
                t_exec0 = time.time()
                self.get_logger().info(f"[{label}] Waiting until execution finished...")
                self.moveit2.wait_until_executed()
                t_exec1 = time.time()
                if self.exec_time is None:
                    self.exec_time = t_exec1 - t_exec0
            except Exception as e:
                self.get_logger().warn(f"[{label}] Waiting for execution failed: {e}")
                return False

        return True
    
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

        # run the main sequence
        try:
            self.start_sequence()
        except Exception as e:
            self.get_logger().error(f"Start sequence failed: {e}")

        # prevent re-running
        self._start_move = lambda: None

    # main sequence 
    def start_sequence(self):
        self.get_logger().info("Waiting for gripper action server…")
        self.gripper_client.wait_for_server()

        self.open_gripper()
        time.sleep(self.grip_exec_delay * 5)
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
            self.get_logger().info(self.GREEN + "-------------------------------------------------------" + self.RESET)
            self.get_logger().info(self.GREEN + "Metrics Summary:" + self.RESET)
            self.get_logger().info(self.GREEN + f"  1. Planning Success (bool): {int(self.planning_success)}" + self.RESET)
            self.get_logger().info(self.GREEN + f"  2. Planning time [s]: {self.plan_time:.4f} s" + self.RESET)
            self.get_logger().info(self.GREEN + f"  3. Execution time [s]: {self.exec_time:.4f} s" + self.RESET)
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
    node = PickAndPlace()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()