import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # 1. Setup Configurations
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    robot_ip = LaunchConfiguration("robot_ip", default="192.168.1.10")

    # Build MoveIt Config 
    moveit_config = (
        MoveItConfigsBuilder("gen3_lite_gen3_lite_2f", package_name="kinova_gen3_lite_moveit_config")
        .robot_description(file_path="config/gen3_lite.urdf.xacro", 
                           mappings={
                               "sim_ignition": "false", 
                               "use_fake_hardware": "false",
                               "dof": "6", 
                               "gripper": "gen3_lite_2f",
                               "robot_ip": "192.168.1.10"
                           })
        .robot_description_semantic(file_path="config/gen3_lite.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )

    # 2. Physical Robot Bringup (Kortex Driver)
    kortex_hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('kortex_bringup'), 'launch', 'gen3_lite.launch.py')
        ]),
        launch_arguments={
            'robot_ip': robot_ip,
            'use_sim_time': use_sim_time,
            'launch_rviz': 'false', 
            'use_fake_hardware': 'false',
            'robot_controller': 'joint_trajectory_controller'
        }.items()
    )


    # 4. MoveGroup Node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), {'use_sim_time': use_sim_time}],
    )

    # 5. Unified RViz
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

    # 6. Static Transform from World to Base_link
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        arguments=["--frame-id", "world", "--child-frame-id", "base_link"],
    )

    return LaunchDescription([
        DeclareLaunchArgument("robot_ip", default_value="192.168.1.10"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        kortex_hardware,
        touch_device,
        move_group_node,
        rviz_node,
        static_tf
    ])