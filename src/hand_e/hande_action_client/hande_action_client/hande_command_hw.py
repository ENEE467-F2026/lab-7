#!/usr/bin/env python3

"""Hand-E Gripper Hardware Action Client Interface using ROS2 Actions.

Usage:
    ros2 run hande_action_client hande_command_hw --open
    ros2 run hande_action_client hande_command_hw --close
    ros2 run hande_action_client hande_command_hw

    Press 'o' + Enter to open, 'c' + Enter to close, 'q' + Enter to quit,

Author: Clinton Enwerem
Developed for the course ENEE467: Robotics Projects Laboratory, Fall 2025, University of Maryland, College Park, MD.
"""
import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from gripper_action.action import GripperAction

# Analog presets 
OPEN     = (1,   10,  10)   # pos=1 (open),   speed=10, force=10
CLOSE    = (255, 10,  10)   # pos=255 (close), speed=10, force=10

MIN_POS = 1
MAX_POS = 255


class HandeCommand(Node):
    def __init__(self):
        super().__init__("hande_command_hw_node")

        self.done = False
        self.gripper_client = ActionClient(
            self, GripperAction, 'robotiq_grip_action'
        )

    # Send goal in analog register units, i.e., (pos, speed, force)
    def send_goal(self, goal_tuple, wait_for_server_sec=5.0):

        if not self.gripper_client.wait_for_server(timeout_sec=wait_for_server_sec):
            self.get_logger().error("Gripper action server not available")
            return

        pos, speed, force = goal_tuple

        goal_msg = GripperAction.Goal()
        goal_msg.desired_position = pos
        goal_msg.desired_speed = speed
        goal_msg.desired_force = force

        self.get_logger().info(
            f"Sending: pos={pos}, speed={speed}, force={force}"
        )

        future = self.gripper_client.send_goal_async(goal_msg)
        future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        gh = future.result()
        if not gh.accepted:
            self.get_logger().warn("Gripper goal rejected")
            self.done = True
            return

        self.get_logger().info("Goal accepted")
        result_future = gh.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_result(self, future):
        res = future.result().result
        self.get_logger().info(f"Gripper finished: {res}")
        self.done = True


# 
def main(args=None):
    rclpy.init(args=args)
    node = HandeCommand()

    def wait_until_done():
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
        node.done = False

    try:
        # CLI flags
        if "--open" in sys.argv:
            node.send_goal(OPEN)
            wait_until_done()
            return

        if "--close" in sys.argv:
            node.send_goal(CLOSE)
            wait_until_done()
            return

        # interactive mode
        print("Hand-E Gripper Hardware Interface (Analog 1-255)")
        print("Commands:")
        print("   o = open")
        print("   c = close")
        print("   q = quit")
        print("   any number 1-255 = desired position\n")

        while rclpy.ok():
            inp = input("Command: ").strip().lower()

            if inp in ("q", "quit", "exit"):
                break

            elif inp in ("o", "open"):
                node.send_goal(OPEN)
                wait_until_done()
                continue

            elif inp in ("c", "close"):
                node.send_goal(CLOSE)
                wait_until_done()
                continue

            try:
                pos = int(inp)
                if not (MIN_POS <= pos <= MAX_POS):
                    raise ValueError
                goal = (pos, 10, 10)     # default speed/force
                node.send_goal(goal)
                wait_until_done()
            except ValueError:
                print(f"Invalid: enter a number between {MIN_POS}-{MAX_POS}, or o/c/q.")

    except KeyboardInterrupt:
        print("\nExiting...")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
