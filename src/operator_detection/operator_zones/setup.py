from glob import glob
import os

from setuptools import find_packages
from setuptools import setup

package_name = 'operator_zones'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dmitrii-ognev',
    maintainer_email='dmog1408@gmail.com',
    description='Zone-based operator proximity monitoring for UR5e safety speed regulation',
    license='MIT',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'zones_node = operator_zones.zones_node:main',
        ],
    },
)
