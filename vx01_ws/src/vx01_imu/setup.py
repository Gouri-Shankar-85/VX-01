from setuptools import setup
import os
from glob import glob

package_name = 'vx01_imu'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='VX-01',
    maintainer_email='vx01@robot.local',
    description='VX-01 IMU driver for HiWonder 10-axis IMU',
    license='MIT',
    entry_points={
        'console_scripts': [
            'vx01_imu_node = vx01_imu.vx01_imu_node:main',
        ],
    },
)