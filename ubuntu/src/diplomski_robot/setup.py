from setuptools import find_packages, setup

package_name = 'diplomski_robot'

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
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'safety_filter = diplomski_robot.safety_filter_node:main',
            'point_and_go = diplomski_robot.point_and_go_node:main',
            'moving_robot = diplomski_robot.moving_robot_node:main',
        ],
    },
)
