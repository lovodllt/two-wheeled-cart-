import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_path = get_package_share_directory('two_wheeled_cart')

    # 设置 Gazebo 插件路径
    gazebo_plugin_path = "/opt/ros/humble/lib"
    if 'GAZEBO_PLUGIN_PATH' in os.environ:
        os.environ["GAZEBO_PLUGIN_PATH"] += os.pathsep + gazebo_plugin_path
    else:
        os.environ["GAZEBO_PLUGIN_PATH"] = gazebo_plugin_path

    # URDF 文件路径
    urdf_file_path = os.path.join(pkg_path, "urdf", "mogi_bot.urdf.xacro")

    # 生成的 URDF 文件路径
    generated_urdf_path = "/tmp/mogi_bot_generated.urdf"

    # 预处理 XACRO 文件生成 URDF - 使用 ExecuteProcess
    generate_urdf = ExecuteProcess(
        cmd=['xacro', urdf_file_path, '-o', generated_urdf_path],
        output='screen',
        shell=True  # 使用 shell 确保命令正确执行
    )

    # 机器人描述内容 - 直接从文件读取
    def get_robot_description():
        # 确保文件存在
        if os.path.exists(generated_urdf_path):
            with open(generated_urdf_path, 'r') as f:
                return f.read()
        else:
            return ""

    # 机器人状态发布器
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(
                Command(['xacro ', urdf_file_path]),
                value_type=str
            ),
            'use_sim_time': True
        }],
        output='screen'
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

    # 在 Gazebo 中生成机器人模型
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'my_robot', '-file', generated_urdf_path, '-x', '0.0', '-y', '0.0', '-z', '0.1'],
        output='screen'
    )


    # 关节状态广播器
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
        output="screen",
    )
    # 简单差速控制器
    diff_drive_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["simple_diff_drive_controller", "-c", "/controller_manager"],
        output="screen",
    )

    camera = Node(
        package='yellow_object_detector',
        executable='contour_pose_node',
        name='contour_pose_node',
        output='screen',
        parameters=[{
            'physical_width': 0.5,
            'physical_height': 0.5,
            'h_min': 20,
            'h_max': 40,
            's_min': 100,
            's_max': 255,
            'v_min': 100,
            'v_max': 255
        }]
    )


    # 事件处理器：URDF 生成完成后启动其他节点
    after_generate_urdf = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=generate_urdf,
            on_exit=[
                robot_state_publisher_node,
                gazebo_launch,
                TimerAction(
                    period=3.0,
                    actions=[spawn_entity_node]
                ),
                TimerAction(
                    period=7.0,
                    actions=[joint_state_broadcaster]
                ),
                TimerAction(
                    period=8.0,
                    actions=[diff_drive_controller]
                )
            ]
        )
    )

    launch_description = LaunchDescription()
    launch_description.add_action(generate_urdf)
    launch_description.add_action(after_generate_urdf)

    return launch_description