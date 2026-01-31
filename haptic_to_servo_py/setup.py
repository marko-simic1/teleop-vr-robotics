import os # <--- Added this import
from glob import glob
from setuptools import find_packages, setup

package_name = 'haptic_to_servo_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        # Include all config files
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        # Include rviz config files
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.rviz'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='libor',
    maintainer_email='libor.zima1@gmail.com',
    description='Haptic to Servo control for Kinova Gen3 Lite',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pose_to_twist = haptic_to_servo_py.pose_to_twist:main',
            'controller_servo = haptic_to_servo_py.controller_servo:main',
            'constant_velocity_publisher = haptic_to_servo_py.constant_publisher:main',
        ],
    },
)