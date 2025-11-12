from setuptools import find_packages, setup

package_name = 'hande_action_client'

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
    description='Simple action client for controlling the Hande gripper via ROS2 actions.',
    license='BSD3-Clause',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'hande_command = hande_action_client.hande_command:main'
        ],
    },
)
