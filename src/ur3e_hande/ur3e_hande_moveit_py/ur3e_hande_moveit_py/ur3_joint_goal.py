#!/usr/bin/env python3
"""
ROS 2 node to move the UR3e HandE robot to a specified joint goal using PyMoveIt2.
"""

from threading import Thread
from typing import Optional
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from pymoveit2 import MoveIt2, MoveIt2State
from pymoveit2.robots import ur3e_hande as robot


class Ur3JointGoalNode(Node):
    def __init__(self):
        super().__init__("ur3_joint_goal_node")
        self.robot = robot
        self.group_states = self.robot.get_named_group_states("")

        # Parameters
        self.declare_parameter("synchronous", False)
        self.declare_parameter("cancel_after_secs", 0.0)
        self.declare_parameter("planner_id", "RRTConnectkConfigDefault") # Same as the RRT* algorithm from Part I

        if self.group_states is not None:
                self.declare_parameter("def_group_state", "ur_home")
                self.declare_parameter("group_state", "ur_home")
                self.def_group_state = self.get_parameter("def_group_state").get_parameter_value().string_value
                requested_state = self.get_parameter("group_state").get_parameter_value().string_value or self.def_group_state

                if requested_state not in self.group_states:
                    self.get_logger().warn(
                        f"Requested group_state '{requested_state}' not found in SRDF. Available: {list(self.group_states.keys())}. "
                        f"Falling back to '{self.def_group_state}'."
                    )
                    requested_state = self.def_group_state

                # we use a named configuration from the SRDF file
                self.declare_parameter("joint_positions", value=self.group_states[requested_state])
                self._requested_group_state = requested_state
        else:
            self.get_logger().warn("No named group states found in SRDF; using default joint positions.")
            self.declare_parameter(
                "joint_positions",
                value=[
                    0.0,     # pan
                    -1.5707, # lift
                    0.0,     # elbow
                    -1.5707, # w1
                    0.0,     # w2
                    0.0,     # w3
                ],
            )
        
        # Callback group for MoveIt2
        self._cb_group = ReentrantCallbackGroup()

        # Create MoveIt2 interface
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=robot.joint_names(),
            base_link_name=robot.base_link_name(),
            end_effector_name=robot.end_effector_name(),
            group_name=robot.MOVE_GROUP_ARM,
            callback_group=self._cb_group,
        )
        self.moveit2.planner_id = (
            self.get_parameter("planner_id").get_parameter_value().string_value
        )

        # Scale down speed
        self.moveit2.max_velocity = 0.2
        self.moveit2.max_acceleration = 0.2

        # State
        self._monitoring = False
        self._execution_future = None
        self._cancel_thread: Optional[Thread] = None

        # Store the timer so we can cancel it to ensure one-shot behavior
        self._start_timer = self.create_timer(1.5, self._start_move, callback_group=self._cb_group)

    def _start_move(self):
        # cancel the timer so this runs only once
        try:
            self._start_timer.cancel()
        except Exception:
            pass
        # Read params
        jp = self.get_parameter("joint_positions").get_parameter_value().double_array_value
        synchronous = self.get_parameter("synchronous").get_parameter_value().bool_value
        cancel_after_secs = self.get_parameter("cancel_after_secs").get_parameter_value().double_value
 
        # Ensure we have at least one joint state before sending the motion request
        # to avoid repeated "Joint states are not available yet!" warnings.

        try:
            self.get_logger().info(f"Moving to {{joint_positions: {list(jp)}}}")
        except Exception:
            self.get_logger().info(f"Moving to {{joint_positions: {list(jp)}}}")
        wait_logged = False
        start = self.get_clock().now()
        rate = self.create_rate(20)
        while rclpy.ok() and self.moveit2.joint_state is None:
            if not wait_logged:
                self.get_logger().info("Waiting for joint states before sending motion request...")
                wait_logged = True
    
            rate.sleep() # Give some time for subscriptions to populate
            if (self.get_clock().now() - start).nanoseconds / 1e9 > 3.0:
                break

        # Execute motion request
        self.moveit2.move_to_configuration(jp)

        if synchronous:
            self.get_logger().info("Waiting until execution finished (synchronous mode)...")
            self.moveit2.wait_until_executed()
            self.get_logger().info("Execution finished.")
            rclpy.shutdown()
        else:
            # Start monitoring for EXECUTING state
            self.get_logger().info("Waiting for execution to start (asynchronous mode)...")
            if not self._monitoring:
                self._monitoring = True
                self.create_timer(0.1, self._monitor_callback, callback_group=self._cb_group)
            self._cancel_after_secs = cancel_after_secs
        self._start_move = lambda: None

    def _monitor_callback(self):
        state = self.moveit2.query_state()
        self.get_logger().debug(f"Current State: {state}")
        if state == MoveIt2State.EXECUTING and self._execution_future is None:
            self.get_logger().info("Execution started.")
            fut = self.moveit2.get_execution_future()
            self._execution_future = fut

            if self._cancel_after_secs > 0.0:
                self.get_logger().info(f"Will cancel after {self._cancel_after_secs} seconds.")
                self._cancel_thread = Thread(
                    target=self._delayed_cancel, args=(self._cancel_after_secs,), daemon=True
                )
                self._cancel_thread.start()

            # Attach done callback
            fut.add_done_callback(self._execution_done)

            # Stop further monitoring
            self._monitoring = False

    def _delayed_cancel(self, delay: float):
        time.sleep(delay)
        self.get_logger().info("Cancelling goal (delayed cancel).")
        self.moveit2.cancel_execution()

    def _execution_done(self, future):
        try:
            result = future.result()
            status = getattr(result, "status", str(result))
            error_code = getattr(getattr(result, "result", None), "error_code", None)
            self.get_logger().info(f"Execution done.") 
        except Exception as e:
            self.get_logger().error(f"Execution finished with exception: {e}")
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = Ur3JointGoalNode()

    executor = rclpy.executors.MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    node.get_logger().info("Waiting 1s for MoveIt2 / topics to initialize...")
    node.create_rate(1.0).sleep()  # sleep 1 second

    try:
        # Keep main thread alive until Ctrl-C
        while rclpy.ok():
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            executor.shutdown()
            executor_thread.join(timeout=1.0)
        except Exception:
            pass
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
