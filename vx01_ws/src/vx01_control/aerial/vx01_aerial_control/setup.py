from setuptools import setup, find_packages

package_name = 'vx01_aerial_control'

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
    maintainer='Gouri Shankar',
    maintainer_email='sskzm5585@gmail.com',
    description='VX-01 drone flight control — Python MAVROS nodes',
    license='MIT',
    entry_points={
        'console_scripts': [
            'aerial_controller_node = vx01_aerial_control.aerial_controller_node:main',
            'drone_flight_manager  = vx01_aerial_control.drone_flight_manager:main',
            'drone_teleop = vx01_aerial_control.drone_teleop:main',
        ],
    },
)
