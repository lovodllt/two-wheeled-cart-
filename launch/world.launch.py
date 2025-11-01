import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取包目录
    pkg_path = get_package_share_directory('two_wheeled_cart')

    # 文件路径
    urdf_file = os.path.join(pkg_path, 'urdf', 'two_wheel_cart.urdf')
    world_file = os.path.join(pkg_path, 'worlds', 'indoor.world')

    # 启动 Gazebo
    gazebo_process = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file],
        output='screen'
    )

    # 在 Gazebo 中生成机器人（延迟执行）
    spawn_robot = ExecuteProcess(
        cmd=[
            'bash', '-c',
            f'sleep 5 && gz service -s /world/my_apartment/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 3000 --req "pose: {{position: {{x: 0, y: 0, z: 0.5}}}}, sdf_filename: \\"{urdf_file}\\", name: \\"two_wheel_cart\\""'
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo_process,
        spawn_robot,
    ])