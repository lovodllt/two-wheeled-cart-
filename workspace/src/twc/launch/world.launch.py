import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_twc = get_package_share_directory('twc')
    world_path = os.path.join(pkg_twc, 'worlds', 'indoor.world')
    urdf_path = os.path.join(pkg_twc, 'urdf', 'two_wheel_cart.urdf')

    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([
        ExecuteProcess(
            cmd=['gz', 'sim', world_path],
            output='screen'
        ),

        TimerAction(
            period=5.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'run', 'ros_gz_sim', 'create',
                         '-file', urdf_path, '-name', 'two_wheeled_robot'],
                    output='screen'
                ),
            ]
        ),

        TimerAction(
            period=7.0,
            actions=[
                Node(
                    package='controller_manager',
                    executable='ros2_control_node',
                    parameters=[{
                        'use_sim_time': True,
                        'robot_description': robot_description
                    }],
                    output='screen',
                    name='controller_manager'
                ),
            ]
        ),

        TimerAction(
            period=9.0,
            actions=[
                Node(
                    package='robot_state_publisher',
                    executable='robot_state_publisher',
                    name='robot_state_publisher',
                    parameters=[{
                        'use_sim_time': True,
                        'robot_description': robot_description
                    }],
                    output='screen'
                ),
            ]
        ),

        TimerAction(
            period=11.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'control', 'load_controller', 'joint_state_controller'],
                    output='screen'
                ),
            ]
        ),

        TimerAction(
            period=13.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'control', 'set_controller_state',
                         'joint_state_controller', 'active'],
                    output='screen'
                ),
            ]
        ),
    ])