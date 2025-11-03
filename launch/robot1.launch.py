import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnProcessStart
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
    urdf_file_path1 = os.path.join(pkg_path, "urdf", "mogi_bot.urdf.xacro")


    # 生成的 URDF 文件路径
    generated_urdf_path1 = "/tmp/mogi_bot_generated.urdf"


    # 预处理 XACRO 文件生成 URDF - 机器人1
    generate_urdf1 = ExecuteProcess(
        cmd=['xacro', urdf_file_path1, '-o', generated_urdf_path1],
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

    # 机器人1状态发布器
    robot_state_publisher_node1 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='robot1',
        parameters=[{
            'robot_description': ParameterValue(
                Command(['xacro ', urdf_file_path1]),
                value_type=str
            ),
            'use_sim_time': True,
            'frame_prefix': 'robot1/'
        }],
        output='screen'
    )

    # 在 Gazebo 中生成机器人1模型
    spawn_entity_node1 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'mogi_bot1',
            '-file', generated_urdf_path1,
            '-x', '0.0', '-y', '0.0', '-z', '0.1',
            '-robot_namespace', 'robot1'
        ],
        output='screen'
    )


    # 机器人1控制器
    joint_state_broadcaster1 = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "robot1/controller_manager"],
        output="screen",
    )

    diff_drive_controller1 = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["simple_diff_drive_controller", "-c", "robot1/controller_manager"],
        output="screen",
    )



    # 事件处理器：依次启动两个机器人
    after_generate_urdf1 = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=generate_urdf1,
            on_exit=[
                robot_state_publisher_node1,
                gazebo_launch,
                TimerAction(
                    period=2.0,
                    actions=[spawn_entity_node1]
                )
            ]
        )
    )



    after_spawn_robot1 = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity_node1,
            on_exit=[
                TimerAction(
                    period=3.0,
                    actions=[joint_state_broadcaster1,diff_drive_controller1]
                )
            ]
        )
    )






    launch_description = LaunchDescription()
    launch_description.add_action(generate_urdf1)
    launch_description.add_action(after_generate_urdf1)

    launch_description.add_action(after_spawn_robot1)

    return launch_description