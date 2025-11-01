import os
from sys import executable

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    os.environ['ROS_DOMAIN_ID'] = '0'

    pkg_path = get_package_share_directory('two_wheeled_cart')

    # 设置 Gazebo 模型路径
    gazebo_models_path = os.path.join(pkg_path, 'models')
    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ["GAZEBO_MODEL_PATH"] += os.pathsep + gazebo_models_path
    else:
        os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path

    # URDF 文件路径
    urdf_file = os.path.join(pkg_path, "urdf", "mogi_bot.urdf")
    print(f"URDF 文件路径: {urdf_file}")

    # 方法1：使用 xacro 解析 URDF（推荐）
    robot_description_content = ParameterValue(
        Command([
            FindExecutable(name='xacro'), ' ', urdf_file
        ]),
        value_type=str
    )

    # 启动 Gazebo
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('gazebo_ros'),
            '/launch/gazebo.launch.py'
        ]),
        launch_arguments={
            'world': os.path.join(pkg_path, 'worlds', 'indoor.world'),
            'verbose': 'false',
            'pause': 'false',
            'gui': 'true'
        }.items()
    )

    # 机器人状态发布器 - 发布 /robot_description 话题
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': True,
            'publish_frequency': 10.0  # 设置发布频率
        }]
    )

    # 在 Gazebo 中生成机器人模型
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'my_robot',
            '-file', urdf_file,
            '-x', '0.0', '-y', '0.0', '-z', '1.0', '-Y', '0.0'
        ],
        output='screen'
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=['joint_state']
    )

    diff_drive_spawner= Node(
        package="controller_manager",
        executable="spawner",
        arguments=['diff_drive']
    )

    launch_description = LaunchDescription()

    # 添加节点
    launch_description.add_action(gazebo_launch)
    launch_description.add_action(robot_state_publisher_node)

    # 延迟生成机器人实体（等待 Gazebo 完全启动）
    launch_description.add_action(TimerAction(
        period=5.0,
        actions=[spawn_entity_node]
    ))
    launch_description.add_action(joint_broad_spawner)
    launch_description.add_action(diff_drive_spawner)

    return launch_description