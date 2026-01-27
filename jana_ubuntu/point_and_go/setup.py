from setuptools import find_packages, setup

package_name = 'point_and_go'

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
    maintainer='jana',
    maintainer_email='jana@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'point_and_go = point_and_go.point_and_go_node:main',
            'moving_robot = point_and_go.moving_robot_node:main',
            'safety_filter = point_and_go.safety_filter_node:main',
        ],
    },
)
