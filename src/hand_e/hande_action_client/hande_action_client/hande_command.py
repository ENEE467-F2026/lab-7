#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import sys
from sensor_msgs.msg import JointState
from control_msgs.action import ParallelGripperCommand

OPEN_POS = 0.025
CLOSED_POS = 0.002

class HandeCommand(Node):
    def __init__(self):
        super().__init__("hande_command_node")
        self.done = False
        ########################################################
        # TODO (Ex.  2a): create action client, gripper_client
        ########################################################
        
        
        ########################################################
    def send_goal(self, 
                  position: float, 
                  wait_for_server_sec: float = 5.0):
        
        if not self.gripper_client.wait_for_server(timeout_sec=wait_for_server_sec):
            self.get_logger().error("Action server not available")
            return False

        # Build the action goal
        ########################################################
        # TODO (Ex.  2b): fill in the goal message
        ########################################################
        cmd = JointState()
        cmd.position = [None] # <-- MODIFY
        cmd.name = [" "]      # <-- MODIFY
        cmd.velocity = []
        cmd.effort = []

        goal_msg = ParallelGripperCommand.Goal()
        goal_msg.command = None # <-- MODIFY

        send_goal_future = self.gripper_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Goal rejected by server")
            self.done = True
            return
        self.get_logger().info("Goal accepted, waiting for result...")
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self._get_result_callback)

    def _get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Result received.")
        self.done = True
        

def main(argv=None):
    rclpy.init(args=argv)
    client = HandeCommand()

    try:
        while rclpy.ok():
            # Parse optional primitive arguments first
            if "--open" in sys.argv:
                position = OPEN_POS
            if "--close" in sys.argv:
                position = CLOSED_POS
            ########################################################
            # TODO (Ex.  2c): prompt user for input
            # Enter code template here
            ########################################################
            

            ########################################################
            ########################################################
            client.send_goal(position=position)
            while rclpy.ok() and not client.done:
                rclpy.spin_once(client, timeout_sec=0.0)
            
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        print("\nExiting...")
        client.destroy_node()


if __name__ == "__main__":
    main()
