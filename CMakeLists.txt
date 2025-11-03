cmake_minimum_required(VERSION 3.8)
project(two_wheeled_cart)

# 设置 C++ 标准
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

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
find_package(gazebo_plugins REQUIRED)
find_package(ros2_control REQUIRED)
find_package(ros2_controllers REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(simple_diff_drive_controller SHARED
        src/simple_diff_drive_controller.cpp)
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/include)
    target_include_directories(simple_diff_drive_controller PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include>
    )
endif()
ament_target_dependencies(simple_diff_drive_controller
        controller_interface
        hardware_interface
        rclcpp
        rclcpp_lifecycle
        geometry_msgs
        nav_msgs
        tf2_ros
        tf2_geometry_msgs
        std_msgs
        pluginlib)

install(TARGETS simple_diff_drive_controller
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/include)
    install(DIRECTORY include/
            DESTINATION include
    )
endif()

install(FILES simple_diff_drive_controller.xml
        DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY scripts
        DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY urdf
        DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY worlds
        DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
        DESTINATION share/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(controller_interface simple_diff_drive_controller.xml)

ament_package()