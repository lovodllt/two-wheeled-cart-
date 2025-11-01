cmake_minimum_required(VERSION 3.8)
project(two_wheeled_cart)

# 查找依赖包
find_package(ament_cmake REQUIRED)

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

# 如果有配置文件
install(DIRECTORY config
        DESTINATION share/${PROJECT_NAME}
)

ament_package()