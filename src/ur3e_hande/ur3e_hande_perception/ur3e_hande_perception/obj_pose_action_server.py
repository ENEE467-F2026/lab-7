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
ROS 2 Node for table-top plane segmentation and object clustering.
Assumes input from /filtered_cloud (downsampled, cropped point cloud).

Usage:
    ros2 run ur3e_hande_perception obj_pose_action_server

Then call the action server 'get_target_obj_pose' from a client or via command line:
ros2 action send_goal /get_target_obj_pose ur3e_hande_planning_interfaces/action/GetTargetObjPose \ 
"{target_position: {x: 0.29, y: 0.51, z: 0.07}, target_height: 0.13, target_obj_bounds: [0.0, 0.0, 0.0, 0.0]}"

Author: Clinton Enwerem
Developed for the course ENEE467: Robotics Projects Laboratory, Fall 2025, University of Maryland, College Park, MD.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from ur3e_hande_planning_interfaces.action import GetTargetObjPose
from ur3e_hande_planning_interfaces.msg import ObjectMetaData
from geometry_msgs.msg import PoseStamped
import numpy as np


class ObjectPoseServer(Node):
    def __init__(self):
        super().__init__("object_pose_server")

        # internal object cache
        self.objects = []  # list[ObjectMetaData]

        # subscriber to live perception topic
        self.create_subscription(
            ObjectMetaData,
            "/object_metadata",
            self.object_metadata_callback,
            10,
        )

        # Action server
        self.action_server = ActionServer(
            self,
            GetTargetObjPose,
            "get_target_obj_pose",
            self.execute_callback,
        )

        self.get_logger().info("Object Pose Server ready and listening on /object_metadata")

    # subscription callback for maintaining object metadata cache
    def object_metadata_callback(self, msg: ObjectMetaData):
        """Store or update detected object metadata in the cache."""
        for i, obj in enumerate(self.objects):
            if obj.id == msg.id:
                self.objects[i] = msg
                self.get_logger().debug(f"Updated object ID {msg.id}")
                return
        self.objects.append(msg)
        self.get_logger().debug(f"Added new object ID {msg.id}")

    # action callback for handling pose requests
    def execute_callback(self, goal_handle):
        goal = goal_handle.request
        result = GetTargetObjPose.Result()
        feedback = GetTargetObjPose.Feedback()

        feedback.status_text = "Searching for nearest object..."
        goal_handle.publish_feedback(feedback)

        # Prefer direct position matching if provided
        target_pos = np.array([goal.target_position.x,
                            goal.target_position.y,
                            goal.target_position.z])
        target_height = goal.target_height if goal.target_height > 0.0 else 0.13

        best_match = None
        best_score = float('inf')

        for obj in self.objects:
            obj_pos = np.array([
                obj.object_pose.pose.position.x,
                obj.object_pose.pose.position.y,
                obj.object_pose.pose.position.z
            ])
            obj_height = float(obj.dimensions[0]) if len(obj.dimensions) > 0 else 0.0

            spatial_dist = np.linalg.norm(target_pos - obj_pos)
            height_diff = abs(target_height - obj_height)
            score = spatial_dist + 0.5 * height_diff  # weighted distance metric

            if score < best_score:
                best_score = score
                best_match = obj

        if best_match is not None and best_score < 0.2:
            feedback.status_text = f"Matched object (score={best_score:.3f})"
            goal_handle.publish_feedback(feedback)
            result.target_obj_found = 1
            result.target_obj_pose = best_match.object_pose
            goal_handle.succeed()
            self.get_logger().info(f"Matched object {best_match.id} (score={best_score:.3f})")
        else:
            feedback.status_text = "No object within tolerance."
            goal_handle.publish_feedback(feedback)
            result.target_obj_found = 0
            goal_handle.abort()

        return result


    # Helper methods
    def bounds_match(self, bounds_a, bounds_b, tol=0.02):
        """Return True if all dimensions match within tolerance."""
        if len(bounds_a) < 3 or len(bounds_b) < 3:
            return False
        return all(abs(float(a) - float(b)) <= tol for a, b in zip(bounds_a[:3], bounds_b[:3]))

    def find_closest_match(self, target_bounds):
        """Find closest match by Euclidean distance in size space."""
        if not self.objects:
            return None
        diffs = [np.linalg.norm(np.array(obj.bounds[:3]) - np.array(target_bounds[:3])) for obj in self.objects]
        idx = int(np.argmin(diffs))
        return self.objects[idx] if diffs[idx] < 0.05 else None


def main(args=None):
    rclpy.init(args=args)
    node = ObjectPoseServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Object Pose Server.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
