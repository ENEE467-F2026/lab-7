import os
from glob import glob
from setuptools import setup

package_name = 'robotiq_api_wrapper'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py')) + glob(os.path.join(package_name, 'launch', '*.launch.py'))),
        (os.path.join('share', 'gripper_srv', 'srv'), glob('srv/*.srv')),
        (os.path.join('share', 'gripper_srv', 'srv'), glob('srv/*.msg')),
        (os.path.join('share', 'gripper_action', 'action'), glob('action/*.action')),        
        (os.path.join('share', 'gripper_action', 'action'), glob('action/*.msg')),        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='gurovaid, coenwerem',
    author_email='iryna.gurova@gmail.com, enwerem@terpmail.umd.edu',
    description='ROS 2 wrapper for Robotiq Hand-E gripper API',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'gripper_action_server = robotiq_api_wrapper.gripper_action_server:main',
            'gripper_node = robotiq_api_wrapper.gripper_node:main',
            'api_test = robotiq_api_wrapper.api_test:main',
            'gripper_joint_publisher = robotiq_api_wrapper.gripper_joint_publisher:main',
        ],
    },
)