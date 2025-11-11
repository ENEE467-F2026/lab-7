import pygame

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from rclpy.action import ActionClient

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from std_msgs.msg import Float64MultiArray, Int32

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import JointState
from gripper_action.action import GripperAction

import time

import roboticstoolbox as rtb
import numpy as np

import random

ur3e_joints=[
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint"
]

"""
TELEOP POSE =
[-1.4197958151446741, # pan
-1.721734186212057, # lift
-1.2863818407058716, # elbow
-1.7442823849120082, # w1
1.5820831060409546, # w2
0.1378641277551651 # w3
]
"""
class SimpleTeleop(Node):
    """
    """

    def __init__(self):
        """
        """

        super().__init__("simple_teleop_hande")

        # readability
        pub_qos_profile=QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10
        )
        self._pub=self.create_publisher(
            msg_type=Float64MultiArray,
            topic="/forward_velocity_controller/commands",
            qos_profile=pub_qos_profile
        )

        sub_qos_profile=QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10
        )

        gripper_srv_qos_profile=QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10
        )
        self._sub=self.create_subscription(
            msg_type=JointState,
            topic="/joint_states",
            callback=self._joint_state_callback,
            qos_profile=sub_qos_profile
        )

        # gripper action client
        self.f_grip = 5
        self.u_grip = 5
        self.gripper_action_client = ActionClient(
            self,
            GripperAction,
            'robotiq_grip_action'
        )

        self.get_logger().info("Waiting for gripper action server...")
        self.gripper_action_client.wait_for_server(timeout_sec=5.0)
        self.get_logger().info("Gripper action server ready.")

        self._timer_group=MutuallyExclusiveCallbackGroup()

        self._timer = self.create_timer(
            timer_period_sec=0.02, # 50 Hz
            callback=self._run,
            callback_group=self._timer_group
        )

        self._model=rtb.models.UR3()

        self._joint_states=[None]*6

        self._joint_states_combined={}

        pygame.init()

        while pygame.joystick.get_count() < 1:
            print("Please connect controller...")
            time.sleep(1.0)
        
        self._joystick=pygame.joystick.Joystick(0)
        self._joystick.init()
        # LEFT UP/DOWN: ax[1] -> UP[-1., 1.]DOWN
        # LEFT LEFT/RIGHT: ax[0] -> LEFT[-1., 1.]RIGHT
        # LEFT TRIGGER: ax[2] -> OUT[-1., 1.]IN
        # RIGHT UP/DOWN: ax[4] -> UP[-1., 1.]DOWN
        # RIGHT LEFT/RIGHT: ax[3] -> LEFT[-1., 1.]RIGHT
        # RIGHT TRIGGER: ax[5] -> OUT[-1., 1.]IN

        # BUTTONS:
        # LB: 4
        # RB: 5
        # A: 0
        # X: 2
        # Y: 3
        # B: 1

    def num_to_range(self, num, inMin, inMax, outMin, outMax):
        return outMin + ((num - inMin) / (inMax - inMin)) * (outMax - outMin)
    
    def send_gripper_goal(self, goal, done_callback):
        goal_msg = GripperAction.Goal()
        goal_msg.desired_position = goal[0]
        goal_msg.desired_speed = goal[1]
        goal_msg.desired_force = goal[2]

        future = self.gripper_action_client.send_goal_async(goal_msg)
        future.add_done_callback(lambda fut: self.goal_response_callback(fut, done_callback))

    def goal_response_callback(self, future, done_callback):
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().warn('Gripper goal rejected')
                return
            self.get_logger().info('Gripper goal accepted, waiting for result...')
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(lambda fut: self.get_result_callback(fut, done_callback))
        except Exception as e:
            self.get_logger().error(f'Error in goal_response_callback: {e}')

    def get_result_callback(self, future, done_callback):
        try:
            result = future.result().result
            # self.get_logger().info(f'Gripper action completed with result: {result}')
            if done_callback:
                done_callback()
        except Exception as e:
            self.get_logger().error(f'Error in get_result_callback: {e}')

    def _run(self):
        if self._joint_states[0] is None:
            # wait for joint data
            return
        
        pygame.event.pump()

        # for ax in range(self._joystick.get_numaxes()):
        #     print(f"ax: [{ax}]: {self._joystick.get_axis(ax)}")

        
        # get desired velocity
        # xdot
        raw=self._joystick.get_axis(0) # 4
        if abs(raw)<0.1:
            raw=0
        x_dot=raw/2

        # ydot
        raw=-1*self._joystick.get_axis(1) # 0
        if abs(raw)<0.1:
            raw=0
        y_dot=raw/2

        # zdot
        raw=self._joystick.get_axis(4) # 1
        if abs(raw)<0.1:
            raw=0
        z_dot=raw/2

        height=self._model.fkine(self._joint_states).data[0][2,3]
        if height<=0.17 and z_dot<0:
            z_dot=0

        # omega
        raw=-1*self._joystick.get_axis(3)
        if abs(raw)<0.1:
            raw=0.
        omega=raw/2

        omega_2=0
        if self._joystick.get_button(1):
            omega_2=0.5
        elif self._joystick.get_button(2):
            omega_2=-0.5

        ve=np.array([
            x_dot,
            y_dot,
            z_dot,
            omega_2, # flat rotate
            0.,    # fwd back rotate
            omega
        ])
        
        jac=self._model.jacob0(self._joint_states)
        try:
            jac=np.linalg.inv(jac)
        except Exception as e:
            print("singular configuration!")
            return
        
        q_dot=jac@ve
        sc=max(abs(q_dot))
        if sc>0.25:
            q_dot=0.25*q_dot/sc
        # print(q_dot)
        # print(self._model.fkine(self._joint_states).data[0][2,3])

        msg=Float64MultiArray()
        msg.data=list(
            {
                "shoulder_pan_joint":   q_dot[0],
                "shoulder_lift_joint":  q_dot[1],
                "elbow_joint":          q_dot[2],
                "wrist_1_joint":        q_dot[3],
                "wrist_2_joint":        q_dot[4],
                "wrist_3_joint":        q_dot[5],
            }.values()
        )

        self._pub.publish(msg)

        # get gripper position (right trigger)
        raw=self._joystick.get_axis(5)
        position_goal = abs(int(self.num_to_range(raw, -1.0, 1.0, 1, 255)))
        self.send_gripper_goal([position_goal, self.u_grip, self.f_grip], None)

    def _joint_state_callback(self, msg):
        for i, joint in enumerate(ur3e_joints):
            idx=msg.name.index(joint)
            self._joint_states[i]=msg.position[idx]
        for name in msg.name:
            idx=msg.name.index(name)
            self._joint_states_combined[name]=msg.position[idx]

def main(args=None):
    rclpy.init()
    demo = SimpleTeleop()
    
    # use a MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(demo)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        demo.destroy_node()
        rclpy.shutdown()

if __name__=="__main__":
    main()
