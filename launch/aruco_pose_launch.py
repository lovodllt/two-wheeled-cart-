from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'physical_width',
            default_value='0.5',
            description='目标物理宽度（米）'
        ),
        DeclareLaunchArgument(
            'physical_height',
            default_value='0.5',
            description='目标物理高度（米）'
        ),
        DeclareLaunchArgument(
            'world_frame',
            default_value='robot1/base_link',
            description='世界坐标系名称'
        ),
        
        DeclareLaunchArgument('h_min', default_value='20'),
        DeclareLaunchArgument('h_max', default_value='40'),
        DeclareLaunchArgument('s_min', default_value='100'),
        DeclareLaunchArgument('s_max', default_value='255'),
        DeclareLaunchArgument('v_min', default_value='100'),
        DeclareLaunchArgument('v_max', default_value='255'),

        Node(
            package='two_wheeled_cart',
            executable='process_node',  
            name='process_node',
            output='screen',
            parameters=[
                {
                    'physical_width': LaunchConfiguration('physical_width'),
                    'physical_height': LaunchConfiguration('physical_height'),
                    'world_frame': LaunchConfiguration('world_frame'),
                    'h_min': LaunchConfiguration('h_min'),
                    'h_max': LaunchConfiguration('h_max'),
                    's_min': LaunchConfiguration('s_min'),
                    's_max': LaunchConfiguration('s_max'),
                    'v_min': LaunchConfiguration('v_min'),
                    'v_max': LaunchConfiguration('v_max')
                }
            ]
        )
    ])
