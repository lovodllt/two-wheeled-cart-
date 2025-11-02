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

    # 机器人配置 - 三个机器人的名称和位置
    robot_configs = [
        {'name': 'robot1', 'x': '0.0', 'y': '0.0', 'z': '0.1'},
        {'name': 'robot2', 'x': '1.0', 'y': '0.0', 'z': '0.1'},
        {'name': 'robot3', 'x': '0.0', 'y': '1.0', 'z': '0.1'}
    ]

    # 机器人状态发布器 - 为每个机器人创建一个
    robot_state_publisher_nodes = []
    for config in robot_configs:
        robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name=f'robot_state_publisher_{config["name"]}',
            namespace=config["name"],
            parameters=[{
                'robot_description': ParameterValue(
                    Command(['xacro ', urdf_file_path]),
                    value_type=str
                ),
                'frame_prefix': f'{config["name"]}/',
                'use_sim_time': True
            }],
            output='screen'
        )
        robot_state_publisher_nodes.append(robot_state_publisher)

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

    # 在 Gazebo 中生成三个机器人模型
    spawn_entity_nodes = []
    for config in robot_configs:
        spawn_entity = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name=f'spawn_{config["name"]}',
            arguments=[
                '-entity', config["name"],
                '-file', generated_urdf_path,
                '-x', config["x"], '-y', config["y"], '-z', config["z"]
            ],
            output='screen'
        )
        spawn_entity_nodes.append(spawn_entity)

    # 关节状态广播器 - 为每个机器人创建一个
    joint_state_broadcaster_nodes = []
    for config in robot_configs:
        joint_broadcaster = Node(
            package="controller_manager",
            executable="spawner",
            name=f'joint_broadcaster_{config["name"]}',
            arguments=[
                "joint_state_broadcaster",
                "--controller-manager", f"/{config['name']}/controller_manager"
            ],
            output="screen",
        )
        joint_state_broadcaster_nodes.append(joint_broadcaster)

    # 简单差速控制器 - 为每个机器人创建一个
    diff_drive_controller_nodes = []
    for config in robot_configs:
        diff_drive = Node(
            package="controller_manager",
            executable="spawner",
            name=f'diff_drive_{config["name"]}',
            arguments=[
                "simple_diff_drive_controller",
                "--controller-manager", f"/{config['name']}/controller_manager"
            ],
            output="screen",
        )
        diff_drive_controller_nodes.append(diff_drive)

    # 摄像头节点（只需要一个）
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
                gazebo_launch,
                TimerAction(
                    period=3.0,
                    actions=[
                        *robot_state_publisher_nodes,  # 启动所有机器人状态发布器
                        *spawn_entity_nodes           # 生成所有机器人
                    ]
                ),
                TimerAction(
                    period=10.0,
                    actions=[*joint_state_broadcaster_nodes]  # 启动所有关节广播器
                ),
                TimerAction(
                    period=12.0,
                    actions=[*diff_drive_controller_nodes]    # 启动所有差速控制器
                )
            ]
        )
    )

    launch_description = LaunchDescription()
    launch_description.add_action(generate_urdf)
    launch_description.add_action(after_generate_urdf)

    return launch_description