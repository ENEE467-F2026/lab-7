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
Pick-and-Place Metrics Logger
Logs planning and execution performance metrics for UR3e-Hand-E
in both simulation and hardware runs.

Metrics:
  - planning_success
  - planning_time [s]
  - execution_time [s]
  - joint_path_length [rad]
  - ee_position_error [m]
  - ee_orientation_error [rad]

Outputs a CSV summary after each run.

Author: Clinton Enwerem
Developed for the course ENEE467: Robotics Projects Laboratory, Fall 2025, University of Maryland, College Park, MD.
"""

import rclpy
from rclpy.node import Node
from moveit_msgs.msg import MoveGroupActionFeedback
from trajectory_msgs.msg import JointTrajectory
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener
import numpy as np
import csv
import os
from scipy.spatial.transform import Rotation as R


class PnpMetricsLogger(Node):
    def __init__(self):
        super().__init__("pnp_metrics_logger")
        self.declare_parameter("csv_path", "pnp_metrics.csv")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("ee_frame", "tool0")
        self.declare_parameter("planned_goal_pose", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])  # [x,y,z,qx,qy,qz,qw]

        self.base_frame = self.get_parameter("base_frame").value
        self.ee_frame = self.get_parameter("ee_frame").value
        self.csv_path = self.get_parameter("csv_path").value
        self.goal_pose = self.get_parameter("planned_goal_pose").value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Runtime containers
        self.last_traj = None
        self.planning_start_time = None
        self.planning_end_time = None
        self.execution_start_time = None
        self.execution_end_time = None

        # Subscribers
        self.create_subscription(
            MoveGroupActionFeedback,
            "/move_group/feedback",
            self.feedback_callback,
            10,
        )
        self.create_subscription(
            JointTrajectory,
            "/move_group/display_planned_path",
            self.traj_callback,
            10,
        )

        self.get_logger().info(
            f"Metrics logger started; results will be written to {self.csv_path}"
        )

    # callbacks 
    def feedback_callback(self, msg):
        """Capture timestamps from MoveIt feedback."""
        status = msg.feedback.state
        now = self.get_clock().now()
        if status == "PLANNING":
            self.planning_start_time = now
        elif status == "PLANNED":
            self.planning_end_time = now
        elif status == "EXECUTING":
            self.execution_start_time = now
        elif status == "SUCCEEDED":
            self.execution_end_time = now
            self.evaluate_metrics()

    def traj_callback(self, msg):
        """Store the latest planned trajectory for later analysis."""
        self.last_traj = msg

    # metric evaluation
    def evaluate_metrics(self):
        """Compute and log metrics once a plan finishes."""
        try:
            plan_time = (
                self.planning_end_time - self.planning_start_time
            ).nanoseconds * 1e-9
        except Exception:
            plan_time = float("nan")

        try:
            exec_time = (
                self.execution_end_time - self.execution_start_time
            ).nanoseconds * 1e-9
        except Exception:
            exec_time = float("nan")

        path_len = self.compute_path_length(self.last_traj)
        pos_err, ang_err = self.compute_pose_error()

        self.get_logger().info(
            f"Plan: {plan_time:.3f}s | Exec: {exec_time:.3f}s | "
            f"Path: {path_len:.3f} | PosErr: {pos_err:.4f} m | OriErr: {ang_err:.2f}°"
        )

        self.write_csv_row(
            {
                "planning_time": plan_time,
                "execution_time": exec_time,
                "joint_path_length": path_len,
                "ee_position_error": pos_err,
                "ee_orientation_error": ang_err,
            }
        )

    # helper methods
    def compute_path_length(self, traj_msg):
        if not traj_msg or not traj_msg.points:
            return float("nan")
        pts = traj_msg.points
        total = 0.0
        for i in range(len(pts) - 1):
            q1 = np.array(pts[i].positions)
            q2 = np.array(pts[i + 1].positions)
            total += np.linalg.norm(q2 - q1)
        return total

    def compute_pose_error(self):
        """Compare actual vs planned EE pose."""
        try:
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                self.base_frame, self.ee_frame, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0)
            )
            t = transform.transform.translation
            q = transform.transform.rotation

            actual_pos = np.array([t.x, t.y, t.z])
            actual_quat = np.array([q.x, q.y, q.z, q.w])

            goal_pos = np.array(self.goal_pose[:3])
            goal_quat = np.array(self.goal_pose[3:])

            pos_err = np.linalg.norm(goal_pos - actual_pos)

            q1 = R.from_quat(goal_quat)
            q2 = R.from_quat(actual_quat)
            ang_err = (q2 * q1.inv()).magnitude() * 180.0 / np.pi

            return pos_err, ang_err
        except Exception as e:
            self.get_logger().warn(f"Pose error computation failed: {e}")
            return float("nan"), float("nan")

    def write_csv_row(self, data_dict):
        write_header = not os.path.exists(self.csv_path)
        with open(self.csv_path, "a", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=list(data_dict.keys()))
            if write_header:
                writer.writeheader()
            writer.writerow(data_dict)


def main(args=None):
    rclpy.init(args=args)
    node = PnpMetricsLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
