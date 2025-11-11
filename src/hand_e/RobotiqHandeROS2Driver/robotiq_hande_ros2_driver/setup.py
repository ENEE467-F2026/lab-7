import os
from glob import glob
from setuptools import setup

package_name = 'robotiq_hande_ros2_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', 'gripper_srv', 'srv'), glob('srv/*.srv')),
        (os.path.join('share', 'gripper_srv', 'srv'), glob('srv/*.msg')),
        (os.path.join('share', 'gripper_action', 'action'), glob('action/*.action')),        
        (os.path.join('share', 'gripper_action', 'action'), glob('action/*.msg')),        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gurovaid',
    maintainer_email='iryna.gurova@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'gripper_action_server = robotiq_hande_ros2_driver.gripper_action_server:main',
            'gripper_node = robotiq_hande_ros2_driver.gripper_node:main',
            'test = robotiq_hande_ros2_driver.test:main',
            'gripper_joint_publisher = robotiq_hande_ros2_driver.gripper_joint_publisher:main',
            'simple_teleop_hande = robotiq_hande_ros2_driver.simple_teleop_hande:main',
            'data_logger = robotiq_hande_ros2_driver.data_logger:main',
        ],
    },
)
