import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_bme_gazebo_basics = get_package_share_directory('two_wheeled_cart')

    gazebo_models_path, ignore_last_dir = os.path.split(pkg_bme_gazebo_basics)
    os.environ["GZ_SIM_RESOURCE_PATH"] += os.pathsep + gazebo_models_path

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

    # 定义URDF/Xacro文件路径
    urdf_file_path = PathJoinSubstitution([
        pkg_bme_gazebo_basics,
        "urdf",
        LaunchConfiguration('model')
    ])

    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bme_gazebo_basics, 'launch', 'world.launch.py'),
        ),
        launch_arguments={
            'world': LaunchConfiguration('world'),
        }.items()
    )

    # 关键修复：添加robot_state_publisher节点
    robot_description_content = ParameterValue(
        Command(['xacro ', urdf_file_path]),
        value_type=str
    )

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

    # Launch rviz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_bme_gazebo_basics, 'rviz', 'rviz.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[{'use_sim_time': True}]
    )

    # Spawn the URDF model - 使用TimerAction延迟执行
    spawn_urdf_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-file", "/home/xuncheng/ws/install/two_wheeled_cart/share/two_wheeled_cart/urdf/mogi_bot.urdf",
            "-name", "my_robot",
            "-x", "0.0", "-y", "0.0", "-z", "1.0", "-Y", "0.0"
        ],
        output="screen",
        parameters=[{'use_sim_time': True}]
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state"],
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive"],
    )
    # controller_manager = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["diff_drive_controller", "-c", "/controller_manager"],
    #     output="screen"
    # )

    # 修复参数桥接配置
    # gz_bridge_node = Node(
    #     package="ros_gz_bridge",
    #     executable="parameter_bridge",
    #     arguments=[
    #         "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
    #         "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
    #         "/odom@nav_msgs/msg/Odometry]gz.msgs.Odometry"
    #     ],
    #     output="screen",
    #     parameters=[{'use_sim_time': True}]
    # )

    # 如果需要关节状态，单独桥接
#     joint_states_bridge = Node(
#     package="ros_gz_bridge",
#     executable="parameter_bridge",
#     arguments=["/model/my_robot/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model"],
#     output="screen",
#     remappings=[
#         ('/model/my_robot/joint_state', '/joint_states')
#     ],
#     parameters=[{'use_sim_time': True}]
# )



    launchDescriptionObject = LaunchDescription()

    launchDescriptionObject.add_action(rviz_launch_arg)
    launchDescriptionObject.add_action(world_arg)
    launchDescriptionObject.add_action(model_arg)
    launchDescriptionObject.add_action(world_launch)
    launchDescriptionObject.add_action(robot_state_publisher_node)
    launchDescriptionObject.add_action(rviz_node)
    # launchDescriptionObject.add_action(gz_bridge_node)
    # launchDescriptionObject.add_action(joint_states_bridge)

    # 关键：使用TimerAction延迟生成机器人
    launchDescriptionObject.add_action(TimerAction(
        period=5.0,  # 等待5秒让Gazebo完全启动
        actions=[spawn_urdf_node]
    ))
    launchDescriptionObject.add_action(joint_broad_spawner)
    launchDescriptionObject.add_action(diff_drive_spawner)
    # launchDescriptionObject.add_action(controller_manager)

    return launchDescriptionObject