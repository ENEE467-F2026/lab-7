from typing import List, Dict, Tuple, Union

# @coenwerem: adding these in for ENEE467; does not follow the regular schema of PyMoveIt2
# They are needed for robot model loading and kinematics computations
import os, yaml
import itertools
import xml.etree.ElementTree as ET
from ament_index_python import get_package_share_directory
from roboticstoolbox.robot.ERobot import ERobot
import subprocess
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import transforms3d as t3d
import numpy as np
from rclpy.node import Node
from numpy.typing import NDArray
import spatialmath as sm

ROBOT_NAME = os.path.basename(__file__).strip().split('.')[0]
MOVEIT_CONFIG_PKG_SHARE_DIR = get_package_share_directory(ROBOT_NAME + "_moveit_config")
DESCRIPTION_FILE = MOVEIT_CONFIG_PKG_SHARE_DIR + "/config/" + ROBOT_NAME + ".urdf.xacro"
JOINT_LIMITS_FILE = MOVEIT_CONFIG_PKG_SHARE_DIR + "/config/"+"joint_limits.yaml"
SRDF_FILE = MOVEIT_CONFIG_PKG_SHARE_DIR + "/config/"+ROBOT_NAME+".srdf"

# SRDF 
TREE = ET.parse(SRDF_FILE)
ROOT = TREE.getroot()

################################
# regular pymoveit2 schema begin
################################
MOVE_GROUP_ARM: str = "arm"

prefix: str = ""

def joint_names(prefix: str = prefix) -> List[str]:
    return [
        prefix + "joint1",
        prefix + "joint2",
        prefix + "joint3"
    ]

def base_link_name(prefix: str = prefix) -> str:
    return prefix + "base_link"

def end_effector_name(prefix: str = prefix) -> str:
    return prefix + "tool_link"
################################
# regular pymoveit2 schema end
################################

def world_frame(prefix: str = prefix) -> str:
    return prefix + "world"

def process_xacro(xacro_file_path: str) -> str:
    """Process a xacro file and return the resulting XML string."""
    urdf_out = "/tmp/robot_3r.urdf"
    try:
        subprocess.run(["xacro", xacro_file_path, "-o", urdf_out], check=True)
    except subprocess.CalledProcessError as e:
        raise RuntimeError(f"Xacro processing failed: {e}")
    return urdf_out

def get_joint_limits():
    try:
        joint_limits = []
        JOINT_NAMES = joint_names("")
        
        with open(JOINT_LIMITS_FILE) as file:
            joint_data = yaml.safe_load(file)

            for joint_name in JOINT_NAMES:
                min_pos_val = joint_data["joint_limits"][joint_name]["min_position"]
                max_pos_val = joint_data["joint_limits"][joint_name]["max_position"]

                if min_pos_val is not None and max_pos_val is not None:
                    joint_limits.append(tuple([min_pos_val, max_pos_val]))

        return joint_limits
    except FileNotFoundError:
        print(f"Error: The file {JOINT_LIMITS_FILE} was not found.")
    except yaml.YAMLError as e:
        print(f"Error while parsing YAML file: {e}.")

def get_named_group_states(prefix: str = prefix) -> Dict[str, List[float]]:
    named_group_states = {}
    for group_state in TREE.findall('group_state'):
        if group_state.get('group') == MOVE_GROUP_ARM:
            named_group_states[prefix+group_state.get('name')] = [float(joint.get('value')) for joint in group_state.findall('joint')]
    return named_group_states

def get_link_names(prefix: str = prefix) -> List[str]:

    return [
        prefix + "base_link",
        prefix + "link0",
        prefix + "link1",
        prefix + "link2",
        prefix + "link3",
        # prefix + "tool_link"
    ]

def get_disabled_collision_pairs(prefix: str=prefix) -> List[Tuple[str]]:
    collision_pairs = []
    for disable_collision in TREE.findall('disable_collisions'):
        collision_pairs.append(tuple([
            prefix+str(disable_collision.get('link1')),
            prefix+str(disable_collision.get('link2'))
        ]))
    return collision_pairs

def get_collision_pairs(prefix: str = prefix) -> List[Tuple[str]]:
    enabled_pairs = []
    disabled_pairs = get_disabled_collision_pairs(prefix)
    link_names = get_link_names(prefix)
    for a, b in itertools.combinations(link_names, 2):
        if (a, b) not in disabled_pairs and (b, a) not in disabled_pairs:
            enabled_pairs.append((a, b))
    return enabled_pairs

def get_rtb_model() -> ERobot:
    """Return the robot model as a Robotics Toolbox ERobot instance."""
    urdf_path = process_xacro(DESCRIPTION_FILE)

    class RobotModel(ERobot):
        def __init__(self):
            links, name, urdf_string, urdf_fp  = super().URDF_read(urdf_path)
            
            self._links = links
            super().__init__(
                links,
                name=name.upper(),
                manufacturer="Custom",
                urdf_string=urdf_string,
                urdf_filepath=urdf_fp
            )
            
            named_group_states = get_named_group_states(prefix="")
            for group_state in named_group_states:
                if "ready" in group_state:
                    group_state_config = named_group_states[group_state]
                    break
            else:
                group_state_config = list(named_group_states.values())[0]
            self.qr = group_state_config
            self.qz = [0]*self.n

            self.addconfiguration("qr", self.qr)
            self.addconfiguration("qz", self.qz)

        @staticmethod
        def load_my_path():
                os.chdir(os.path.dirname(__file__))
    return RobotModel()

def se3_to_pose_stamped(se3: sm.SE3, node_obj:Node, frame_id:str="base_link") -> PoseStamped:
    """Convert RTB SE3 pose to ROS PoseStamped message"""
    pose_msg = PoseStamped()
    pose_msg.header = Header()
    pose_msg.header.frame_id = frame_id
    pose_msg.header.stamp = node_obj.get_clock().now().to_msg()

    T = se3.A  # 4x4 transformation matrix
    pose_msg.pose.position.x = T[0, 3]
    pose_msg.pose.position.y = T[1, 3]
    pose_msg.pose.position.z = T[2, 3]

    # Extract quaternion from rotation matrix
    quat = t3d.quaternions.mat2quat(T[:3, :3])  # returns (w, x, y, z)
    pose_msg.pose.orientation.w = quat[0]
    pose_msg.pose.orientation.x = quat[1]
    pose_msg.pose.orientation.y = quat[2]
    pose_msg.pose.orientation.z = quat[3]

    return pose_msg

def pose_stamped_to_se3(pose_stamped_obj: PoseStamped, node_obj:Union[Node, None]=None, base_frame_id:Union[str, None]="base_link") -> NDArray:
    trans = [pose_stamped_obj.pose.position.x, pose_stamped_obj.pose.position.y, pose_stamped_obj.pose.position.z]
    quat = [pose_stamped_obj.pose.orientation.w, pose_stamped_obj.pose.orientation.x, pose_stamped_obj.pose.orientation.y, pose_stamped_obj.pose.orientation.z]
    R_mat = t3d.quaternions.quat2mat(np.array(quat))
    # Build the 4x4 transformation matrix from R_mat and trans
    T = np.eye(4)
    T[:3, :3] = R_mat
    T[:3, 3] = trans
    return T

def compute_fk(robot_model:ERobot, q: NDArray, link_names:Union[None, List]=None, base_frame:Union[str, None]="base_link", node_obj:Union[Node, None] = None) -> List[PoseStamped]:
    """Return list of PoseStamped for each link in robot_model.fkine_all(q)."""
    robot_model.q = q
    # compute transforms for all links 
    transforms = robot_model.fkine_all(q)

    # map each robot link name to its transform (
    name_to_pose = {
        link.name: se3_to_pose_stamped(T, node_obj=node_obj, frame_id=base_frame)
        for link, T in zip(robot_model.links, transforms)
    }
    # confirm counts 
    if node_obj is not None:
        try:
            node_obj.get_logger().debug(f"compute_fk: transforms={len(transforms)}, links={len(robot_model.links)}, requested={link_names}")
        except Exception:
            pass

    # choose which link names to return
    req_names = link_names if link_names is not None else get_link_names("")
    poses = [name_to_pose[name] for name in req_names if name in name_to_pose]
    return poses

def compute_ik(robot_model: ERobot, 
               pose_stamped: PoseStamped, 
               init_guess: Union[NDArray, None],
               base_frame:Union[str, None]="base_link", 
               tool_frame:Union[str, None]="tool_link", 
               ik_method: Union[str, None]="LM",
               error_weights: Union[List, None]=[1,1,1,1,1,1],
               max_iter:Union[int, None]=1000,
               max_searches:Union[int, None]=10,
               e_tol:Union[float, None]=1e-6) -> Union[Dict[str, Union[int, float, NDArray]], None]:
    """Return joint configuration q that achieves the desired pose."""
    try:
        T = pose_stamped_to_se3(pose_stamped, base_frame_id=base_frame)
        # Choose IK method
        if ik_method == "LM":
            ik_sol = robot_model.ikine_LM(
                sm.SE3(T), 
                mask=error_weights, # weights for x,y,z,rx,ry,rz Cartesian error
                ilimit=max_iter, 
                slimit=max_searches,
                tol=e_tol,
                q0=init_guess
            )
        elif ik_method == "GN":
            ik_sol = robot_model.ikine_GN(
                sm.SE3(T), 
                mask=error_weights, # weights for x,y,z,rx,ry,rz Cartesian error
                ilimit=max_iter, 
                slimit=max_searches,
                tol=e_tol,
                q0=init_guess,
                pinv=True
            )
        elif ik_method == "NR":
            ik_sol = robot_model.ikine_NR(
            sm.SE3(T),
            mask=error_weights,
            ilimit=max_iter,
            slimit=max_searches,
            tol=e_tol,
            q0=init_guess,
            pinv=True
            )
        elif ik_method == "QP":
            ik_sol = robot_model.ikine_QP(
            sm.SE3(T),
            mask=error_weights,
            ilimit=max_iter,
            slimit=max_searches,
            tol=e_tol,
            q0=init_guess
            )
        else:
            raise ValueError(f"Unknown IK method: {ik_method}")
        if ik_sol.success:
            return {"q": ik_sol.q, 
                    "num_iterations": ik_sol.iterations, 
                    "err": ik_sol.residual}
        else:
            # print(f"IK did not converge: {ik_sol.reason}")
            return None
    except Exception as e:
        print(f"IK computation error: {e}")
        return None