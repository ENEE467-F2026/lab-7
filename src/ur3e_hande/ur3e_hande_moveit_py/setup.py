
import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'ur3e_hande_moveit_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Clinton Enwerem',
    maintainer_email='enwerem@terpmail.umd.edu',
    description='Python nodes for motion planning and control on the UR3e-Hande-E using PyMoveIt2, ros2_control, and a GripperAction node for the Robotiq Hand-E.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pnp_demo = ur3e_hande_moveit_py.pnp_demo:main',
            'ur3_joint_goal = ur3e_hande_moveit_py.ur3_joint_goal:main',
        ],
    },
)
