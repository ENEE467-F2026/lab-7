
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
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py')) + glob(os.path.join(package_name, 'launch', '*.launch.py')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Clinton Enwerem',
    maintainer_email='enwerem@terpmail.umd.edu',
    description='Python nodes for motion planning and control on the UR3e-Hande-E using PyMoveIt2 and control.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pnp_demo_sim = ur3e_hande_moveit_py.pnp_demo_sim:main',
            'pnp_demo_ik = ur3e_hande_moveit_py.pnp_demo_ik:main',
            'pnp_demo = ur3e_hande_moveit_py.pnp_demo:main',
            'ur3_joint_goal = ur3e_hande_moveit_py.ur3_joint_goal:main'

        ],
    },
)
