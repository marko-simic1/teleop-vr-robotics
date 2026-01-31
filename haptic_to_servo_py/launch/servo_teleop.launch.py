import os
import yaml
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None

def generate_launch_description():
    # 1. Load MoveIt Config - Now explicitly including kinematics and limits
    moveit_config = (
        MoveItConfigsBuilder("gen3_lite", package_name="kinova_gen3_lite_moveit_config")
        .robot_description(file_path="config/gen3_lite.urdf.xacro")
        .robot_description_semantic(file_path="config/gen3_lite.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .to_moveit_configs()
    )

    # 2. Servo Params - Directly using the loaded YAML
    servo_yaml = load_yaml("haptic_to_servo_py", "config/kinova_servo_config.yaml")

    # 3. KORTEX SIM CONTROL (Robot Simulation)
    kortex_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('kortex_bringup'), 'launch', 'kortex_sim_control.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': 'true',
            'launch_rviz': 'false',
            'robot_type': 'gen3_lite',
            'robot_name': 'gen3_lite',
            'dof': '6',
            'gripper': 'gen3_lite_2f'
        }.items()
    )

    # 4. MoveIt Servo Node - Explicitly passing all MoveIt config components
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="servo_node",
        parameters=[
            servo_yaml,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            {'use_sim_time': True}
        ],
        output="screen",
    )

    # 5. Custom Controller Node
    custom_servo_control_node = Node(
        package="haptic_to_servo_py",
        executable="controller_servo",
        name="controller_servo_node",
        parameters=[{'use_sim_time': True}],
        output="screen",
    )

    # 6. RViz2 - Added semantic and kinematics so the MoveIt plugin works
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {'use_sim_time': True}
        ],
    )

    return LaunchDescription([
        kortex_sim,
        servo_node,
        custom_servo_control_node,
        rviz_node
    ])