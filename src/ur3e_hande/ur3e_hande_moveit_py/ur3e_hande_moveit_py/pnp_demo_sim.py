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
import roboticstoolbox as rtb
import spatialmath as sm

from pymoveit2 import MoveIt2
from pymoveit2.robots import ur3e_hande as robot
from control_msgs.action import ParallelGripperCommand
from ur3e_hande_planning_interfaces.action import GetTargetObjPose

ur_approach = [
    -1.8605,         # pan 
    -2.99056,        # lift 
        0.675617,    # elbow 
    -2.00445,        # w1 
        1.61112,     # w2  
    -0.00712473,     # w3  
]

# Alternate joint configurations for cube pick-and-place 
ur_approach_cube = [
    -1.86057,
    -2.99056,
    0.475617,
    -2.00445,
    1.61112,
    -0.00712473,
]

ur_pregrip = [
    -1.83906,
    -3.10465,
    0.656839,
    -2.02065,
    1.61113,
    -0.00707275,
]

# place seed
ur_place = [
    -0.626256,    # pan
    -3.11027,     # lift
     0.681228,    # elbow
    -2.06301,     # w1
     1.47058,     # w2
    -0.00735981,  # w3
]

class PickAndPlaceSim(Node):
    def __init__(self):
        super().__init__("pnp_demo_sim")
        self.rtb_model = rtb.models.UR3()
        self.cb_group = ReentrantCallbackGroup()

        # params
        self.declare_parameter("obj_pos", [])
        self.declare_parameter("target_obj_bounds", [0.3, 0.5, -0.2, 0.2])
        self.declare_parameter("target_height", 0.13)
        self.declare_parameter("target_position", [0.29, 0.51, 0.07])
        self.declare_parameter("pregrasp_z", 0.28)
        self.declare_parameter("lift_z_offset", 0.12)
        self.declare_parameter("place_offset_y", 0.0)
        self.declare_parameter("ee_frame", "tool0")
        self.declare_parameter("grip_exec_delay", 0.9) # s

        self.pregrasp_z = self.get_parameter("pregrasp_z").value
        self.lift_z_offset = self.get_parameter("lift_z_offset").value
        self.place_offset_y = self.get_parameter("place_offset_y").value
        self.target_height = self.get_parameter("target_height").value
        self.target_position = self.get_parameter("target_position").value
        self.ee_frame = self.get_parameter("ee_frame").get_parameter_value().string_value
        self.grip_exec_delay = self.get_parameter("grip_exec_delay").value

        self.declare_parameter("place_pos_arg", [-self.target_position[0], 
                                                 self.target_position[1], 
                                                 self.pregrasp_z])
        self.place_pos_arg = self.get_parameter("place_pos_arg").get_parameter_value().double_array_value

        # synchronous execution: wait for each motion to finish before continuing
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
        self.lift_z = max(self.obj_pos[2] + self.lift_z_offset, self.pregrasp_z + 0.03)
        self.get_logger().info(f"Computed lift_z: {self.lift_z:.3f} m")

        self.approach_pos = [self.obj_pos[0], self.obj_pos[1], self.lift_z]
        self.get_logger().info(f"Approach position: {self.approach_pos}")

        self.grasp_pos = [self.obj_pos[0], self.obj_pos[1], self.pregrasp_z]
        self.get_logger().info(f"Grasp position: {self.grasp_pos}")

        self.place_pos = [float(x) for x in self.place_pos_arg] if self.place_pos_arg else self.compute_place_pos(self.grasp_pos)

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
        self.moveit2.max_velocity = 0.25
        self.moveit2.max_acceleration = 0.25
        self.moveit2.goal_position_tolerance = 0.003
        self.moveit2.goal_orientation_tolerance = 0.01

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
    def _ik_with_yaw_sweep(self, pos_xyz, q0=None, yaw_candidates_deg=((0, 0, 0, 0, 0, 0, 0, 0))): #(0, 45, -45, 90, -90, 135, -135, 180))
        """Try multiple EE yaw angles about +Z to find a valid IK"""
        for yaw_deg in yaw_candidates_deg:
            # q = self.moveit2.compute_ik(pos_xyz, quat_xyzw)
            # choose initial seed for IK
            q0_seed = q0 if q0 is not None else ur_approach
            se_approach = self.rtb_model.fkine(q0_seed, end=self.ee_frame)

            # rtb fallback using the chosen seed's pose
            T_goal = sm.SE3.Trans(x=pos_xyz[0], y=pos_xyz[1], z=pos_xyz[2]) * sm.SE3.Rt(se_approach.R, [0,0,0])
            q_rtb = self.rtb_model.ikine_LM(
                Tep=T_goal,
                q0=q0_seed,
                ilimit=2000,
                end=self.ee_frame,
                tol=1e-1)
            self.get_logger().info(f"RTB IK error norm at yaw {yaw_deg} deg: {q_rtb.residual:.6f}")
            if q_rtb.success:
                self.get_logger().info(f"\x1b[32mRTB IK success at yaw {yaw_deg} deg; q: {q_rtb.q.tolist()}\x1b[0m")
                return q_rtb.q.tolist()
            else:
                self.get_logger().debug(f"IK failed for yaw {yaw_deg} deg")
        return None

    def _plan_then_execute(self, q, label):
        """Plan a trajectory; only execute if valid"""
        if q is None:
            self.get_logger().warn(f"[{label}] No IK solution ;  skipping.")
            return False
        synchronous = self.get_parameter("synchronous").get_parameter_value().bool_value

        try:
            traj = self.moveit2.move_to_configuration(q)
        except Exception as e:
            self.get_logger().warn(f"[{label}] Planning/execution threw: {e}")
            return False

        if traj is not None and not getattr(traj, "joint_trajectory", None):
            self.get_logger().warn(f"[{label}] Planning returned an unexpected result.")

        if synchronous:
            # Wait until execution finishes before continuing 
            try:
                self.get_logger().info(f"[{label}] Waiting until execution finished...")
                self.moveit2.wait_until_executed()
            except Exception as e:
                self.get_logger().warn(f"[{label}] Waiting for execution failed: {e}")
                return False

        return True

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

        # approach
        q_approach = self._ik_with_yaw_sweep(self.approach_pos)
        if not self._plan_then_execute(q_approach, "Approach"):
            self.get_logger().error("Aborting: cannot reach approach pose.")
            return

        # grasp
        # use cube-specific approach as IK seed for grasp
        q_grasp = self._ik_with_yaw_sweep(self.grasp_pos, q0=ur_pregrip)
        if not self._plan_then_execute(q_grasp, "Grasp"):
            self.get_logger().error("Aborting: cannot reach grasp pose.")
            return

        self.open_gripper()
        time.sleep(self.grip_exec_delay)
        self.close_gripper()
        time.sleep(self.grip_exec_delay)

        # lift
        if not self._plan_then_execute(q_approach, "Lift"):
            self.get_logger().error("Aborting: cannot lift after grasp.")
            return

        # place
        q_place = self._ik_with_yaw_sweep(self.place_pos, q0=ur_place)
        if not self._plan_then_execute(q_place, "Place"):
            self.get_logger().error("Aborting: cannot reach place pose.")
            return
        self.open_gripper()
        time.sleep(self.grip_exec_delay)
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
        self.send_gripper_goal(0.000)


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