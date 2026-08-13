from setuptools import find_packages, setup

package_name = 'grasping_teleop'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='kalanaratnayake95@gmail.com',
    description='Keyboard teleoperation and pose-reading utilities for the grasping stack',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'read_pose_node = grasping_teleop.read_pose_node:main',
            'servo_teleop_node = grasping_teleop.servo_teleop_node:main',
        ],
    },
)