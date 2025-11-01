import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    os.environ['ROS_DOMAIN_ID'] = '0'

    pkg_bme_gazebo_basics = get_package_share_directory('two_wheeled_cart')

    # 设置 Gazebo 模型路径
    gazebo_models_path = os.path.join(pkg_bme_gazebo_basics, 'models')
    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ["GAZEBO_MODEL_PATH"] += os.pathsep + gazebo_models_path
    else:
        os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path

    rviz_launch_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Open RViz.'
    )

    world_arg = DeclareLaunchArgument(
        'world', default_value='indoor.world',
        description='Name of the Gazebo world file to load'
    )

    model_arg = DeclareLaunchArgument(
        'model', default_value='mogi_bot.urdf',
        description='Name of the URDF description to load'
    )

    # URDF 文件路径
    urdf_file_path = os.path.join(pkg_bme_gazebo_basics, "urdf", "mogi_bot.urdf")

    # 检查文件是否存在
    if not os.path.exists(urdf_file_path):
        print(f"错误: URDF 文件不存在: {urdf_file_path}")
        return LaunchDescription()

    # 关键修复：使用完整的 xacro 命令，设置 ROS_PACKAGE_PATH
    robot_description_content = ParameterValue(
        Command([
            FindExecutable(name='xacro'), ' ',
            urdf_file_path
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
            'world': os.path.join(pkg_bme_gazebo_basics, 'worlds', 'indoor.world'),
            'verbose': 'false',
            'pause': 'false'
        }.items()
    )

    # 机器人状态发布器
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': True
        }]
    )

    # 控制器管理器节点
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': True
        }],
        output='screen'
    )

    # RViz2 节点
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_bme_gazebo_basics, 'rviz', 'rviz.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # 在 Gazebo 中生成机器人模型
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'my_robot',
            '-file', urdf_file_path,
            '-x', '0.0', '-y', '0.0', '-z', '1.0', '-Y', '0.0'
        ],
        output='screen'
    )

    # 控制器 spawner 节点
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen"
    )

    diff_drive_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller"],
        output="screen"
    )

    launch_description = LaunchDescription()

    # 添加参数
    launch_description.add_action(rviz_launch_arg)
    launch_description.add_action(world_arg)
    launch_description.add_action(model_arg)

    # 添加节点
    launch_description.add_action(gazebo_launch)
    launch_description.add_action(robot_state_publisher_node)
    launch_description.add_action(controller_manager_node)
    launch_description.add_action(rviz_node)

    # 延迟生成机器人实体
    launch_description.add_action(TimerAction(
        period=5.0,
        actions=[spawn_entity_node]
    ))

    # 延迟启动控制器
    launch_description.add_action(TimerAction(
        period=8.0,
        actions=[joint_state_broadcaster, diff_drive_controller]
    ))

    return launch_description