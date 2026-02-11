import os
import yaml
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    with open(absolute_file_path, "r") as file:
        return yaml.safe_load(file)

def generate_launch_description():
    # 1. Setup Configurations
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    
    # Build MoveIt Config (Shared by MoveGroup, Servo, and RViz)
    moveit_config = (
        MoveItConfigsBuilder("gen3_lite", package_name="kinova_gen3_lite_moveit_config")
        .robot_description(file_path="config/gen3_lite.urdf.xacro", 
                           mappings={"sim_ignition": "true", "dof": "6", "gripper": "gen3_lite_2f"})
        .robot_description_semantic(file_path="config/gen3_lite.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )

    # Load Servo YAML
    servo_yaml = load_yaml("haptic_to_servo_py", "config/kinova_servo_config.yaml")

    # 2. Kortex Simulation Bringup
    kortex_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('kortex_bringup'), 'launch', 'kortex_sim_control.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'launch_rviz': 'false', # We launch our own RViz
            'robot_type': 'gen3_lite',
            'dof': '6',
            'gripper': 'gen3_lite_2f'
        }.items()
    )

    # 3. Touch Device (Isolated in Namespace)
    touch_device = GroupAction(
        actions=[
            PushRosNamespace('touch'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(get_package_share_directory('touch_common'), 'launch', 'touch.launch.py')
                ]),
                launch_arguments={'launch_rviz': 'false', 'use_sim_time': use_sim_time}.items()
            ),
        ]
    )

    # 4. MoveGroup Node (For standard planning)
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), {'use_sim_time': use_sim_time}],
    )

    # 5. MoveIt Servo Node (For haptic/real-time control)
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
            {'use_sim_time': use_sim_time}
        ],
        output="screen",
    )

    # 6. Your Custom Haptic Logic Node
    custom_servo_control_node = Node(
        package="haptic_to_servo_py",
        executable="controller_servo",
        name="controller_servo_node",
        parameters=[{'use_sim_time': use_sim_time}],
        output="screen",
    )

    velocity_calc_node = Node(
        package="haptic_to_servo_py",
        executable="pose_to_twist",
        name="pose_to_twist_node",
        parameters=[{'use_sim_time': use_sim_time}],
        output="screen",
    )

    # 7. Unified RViz
    rviz_config_path = os.path.join(get_package_share_directory("kinova_gen3_lite_moveit_config"), "config", "moveit.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config_path],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            {'use_sim_time': use_sim_time}
        ],
    )

    return LaunchDescription([
        kortex_sim,
        touch_device,
        move_group_node,
        servo_node,
        custom_servo_control_node,
        velocity_calc_node,
        rviz_node
    ])