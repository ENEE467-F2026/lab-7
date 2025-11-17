"""
UR3e with Robotiq Hand-E robot definitions and utilities.

Author: Clinton Enwerem
Developed for the course ENEE467: Robotics Projects Laboratory, Fall 2025, University of Maryland, College Park, MD.
Adapted from https://github.com/AndrejOrsula/pymoveit2/blob/main/pymoveit2/robots/ur.py
"""
from typing import List, Dict, Tuple
import os
import yaml
import itertools
import xml.etree.ElementTree as ET
from ament_index_python import get_package_share_directory
import subprocess

MOVE_GROUP_ARM: str = "ur_manipulator"
MOVE_GROUP_GRIPPER: str = "gripper"

prefix: str = ""

OPEN_GRIPPER_JOINT_POSITIONS: List[float] = [0.025, 0.025]
CLOSED_GRIPPER_JOINT_POSITIONS: List[float] = [0.0, 0.0]


def joint_names(prefix: str = prefix) -> List[str]:
    return [
        prefix + "shoulder_pan_joint",
        prefix + "shoulder_lift_joint",
        prefix + "elbow_joint",
        prefix + "wrist_1_joint",
        prefix + "wrist_2_joint",
        prefix + "wrist_3_joint",
    ]


def base_link_name(prefix: str = prefix) -> str:
    return prefix + "base_link"


def end_effector_name(prefix: str = prefix) -> str:
    return prefix + "tool0"


def gripper_joint_names(prefix: str = prefix) -> List[str]:
    return [
        prefix + "gripper_robotiq_hande_left_finger_joint",
        prefix + "gripper_robotiq_hande_right_finger_joint",
    ]

# @coenwerem: adding these in for ENEE467; does not follow the regular schema of PyMoveIt2
# They are needed for robot model loading and kinematics computations
# Derived paths for moveit config and robot description
ROBOT_NAME = os.path.basename(__file__).strip().split('.')[0]
MOVEIT_CONFIG_PKG_SHARE_DIR = get_package_share_directory(ROBOT_NAME + "_moveit_config")
DESCRIPTION_FILE = os.path.join(MOVEIT_CONFIG_PKG_SHARE_DIR, "config", ROBOT_NAME + ".urdf.xacro")
JOINT_LIMITS_FILE = os.path.join(MOVEIT_CONFIG_PKG_SHARE_DIR, "config", "joint_limits.yaml")
SRDF_FILE = os.path.join(MOVEIT_CONFIG_PKG_SHARE_DIR, "config", ROBOT_NAME + ".srdf")

# Parse SRDF if available
try:
    TREE = ET.parse(SRDF_FILE)
    ROOT = TREE.getroot()
except Exception:
    TREE = None
    ROOT = None


def process_xacro(xacro_file_path: str) -> str:
    """Process a xacro file and return the resulting URDF file path."""
    urdf_out = f"/tmp/{ROBOT_NAME}.urdf"
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
            for jn in JOINT_NAMES:
                if jn in joint_data.get("joint_limits", {}):
                    min_pos_val = joint_data["joint_limits"][jn].get("min_position")
                    max_pos_val = joint_data["joint_limits"][jn].get("max_position")
                    if min_pos_val is not None and max_pos_val is not None:
                        joint_limits.append((min_pos_val, max_pos_val))
        return joint_limits
    except FileNotFoundError:
        print(f"Warning: joint limits file not found: {JOINT_LIMITS_FILE}")
        return []
    except Exception as e:
        print(f"Error loading joint limits: {e}")
        return []


def get_named_group_states(prefix: str = prefix) -> Dict[str, List[float]]:
    named_group_states = {}
    if TREE is None:
        return named_group_states
    for group_state in TREE.findall('group_state'):
        if group_state.get('group') == MOVE_GROUP_ARM:
            named_group_states[prefix + group_state.get('name')] = [float(joint.get('value')) for joint in group_state.findall('joint')]
    return named_group_states


def get_link_names(prefix: str = prefix) -> List[str]:
    # Basic UR-like link naming
    return [
        prefix + "base_link",
        prefix + "shoulder_link",
        prefix + "upper_arm_link",
        prefix + "forearm_link",
        prefix + "wrist_1_link",
        prefix + "wrist_2_link",
        prefix + "wrist_3_link",
        prefix + "tool0",
    ]


def get_disabled_collision_pairs(prefix: str = prefix) -> List[Tuple[str]]:
    pairs = []
    if TREE is None:
        return pairs
    for disable_collision in TREE.findall('disable_collisions'):
        pairs.append((prefix + str(disable_collision.get('link1')), prefix + str(disable_collision.get('link2'))))
    return pairs


def get_collision_pairs(prefix: str = prefix) -> List[Tuple[str]]:
    enabled_pairs = []
    disabled_pairs = get_disabled_collision_pairs(prefix)
    link_names = get_link_names(prefix)
    for a, b in itertools.combinations(link_names, 2):
        if (a, b) not in disabled_pairs and (b, a) not in disabled_pairs:
            enabled_pairs.append((a, b))
    return enabled_pairs
