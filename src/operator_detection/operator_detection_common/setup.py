from setuptools import find_packages
from setuptools import setup

package_name = 'operator_detection_common'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dmitrii-ognev',
    maintainer_email='dmog1408@gmail.com',
    description='Shared constants and utilities for the operator_detection pipeline',
    license='MIT',
    extras_require={
        'test': ['pytest'],
    },
)
