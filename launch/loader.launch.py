import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_path = get_package_share_directory('two_wheeled_cart')

    # 设置 Gazebo 插件路径 - 关键修复
    gazebo_plugin_path = "/opt/ros/humble/lib"
    if 'GAZEBO_PLUGIN_PATH' in os.environ:
        os.environ["GAZEBO_PLUGIN_PATH"] += os.pathsep + gazebo_plugin_path
    else:
        os.environ["GAZEBO_PLUGIN_PATH"] = gazebo_plugin_path

    # URDF 文件路径
    urdf_file_path = os.path.join(pkg_path, "urdf", "mogi_bot.urdf")

    robot_description_content = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', urdf_file_path]),
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

    # 机器人状态发布器
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_content, 'use_sim_time': True}],
        output='screen'
    )

    # 在 Gazebo 中生成机器人模型
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'my_robot', '-file', urdf_file_path, '-x', '0.0', '-y', '0.0', '-z', '1.0'],
        output='screen'
    )

    launch_description = LaunchDescription()
    launch_description.add_action(gazebo_launch)
    launch_description.add_action(robot_state_publisher_node)

    # 延迟生成机器人实体
    launch_description.add_action(TimerAction(
        period=5.0,
        actions=[spawn_entity_node]
    ))

    return launch_description