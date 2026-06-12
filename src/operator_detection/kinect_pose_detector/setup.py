from glob import glob
import os

from setuptools import find_packages
from setuptools import setup

package_name = 'kinect_pose_detector'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'models'), glob('models/*.task')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dmitrii-ognev',
    maintainer_email='dmog1408@gmail.com',
    description='MediaPipe-based 3D human pose detection from Kinect v2 for ROS2',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pose_detector_node = kinect_pose_detector.pose_detector_node:main'
        ],
    },
)
