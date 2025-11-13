import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'ur3e_hande_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py')) + glob(os.path.join(package_name, 'launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'rviz'),
            glob(os.path.join('rviz', '*rviz')) + glob(os.path.join(package_name, 'rviz', '*.rviz')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Clinton Enwerem',
    maintainer_email='enwerem@terpmail.umd.edu',
    description='Package for point cloud-based perception for the UR3e robot with Robotiq Hande gripper',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pc_voxel_filter_node = ur3e_hande_perception.pc_voxel_filter_node:main',
            'pc_segmentation_node = ur3e_hande_perception.pc_segmentation_node:main',
            'obj_pose_action_server = ur3e_hande_perception.obj_pose_action_server:main',
        ],
    },
)
