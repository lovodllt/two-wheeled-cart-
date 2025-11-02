cmake_minimum_required(VERSION 3.8)
project(two_wheeled_cart)

# 查找依赖包
find_package(ament_cmake REQUIRED)

# 查找其他依赖
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_ros2_control REQUIRED)
find_package(controller_manager REQUIRED)
find_package(joint_state_broadcaster REQUIRED)
find_package(diff_drive_controller REQUIRED)
find_package(gazebo_plugins REQUIRED)
find_package(ros2_control REQUIRED)
find_package(ros2_controllers REQUIRED)

# 安装 launch 文件
install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}
)

# 安装 URDF 文件
install(DIRECTORY urdf
        DESTINATION share/${PROJECT_NAME}
)

# 安装 world 文件
install(DIRECTORY worlds
        DESTINATION share/${PROJECT_NAME}
)

# 安装配置文件
install(DIRECTORY config
        DESTINATION share/${PROJECT_NAME}
)

ament_package()