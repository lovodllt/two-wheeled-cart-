cmake_minimum_required(VERSION 3.8)
project(twc)

# 查找 ament_cmake 构建工具
find_package(ament_cmake REQUIRED)

# 安装启动文件
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# 安装世界文件
install(DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}
)

# 安装 URDF 文件
install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

# 安装配置文件（如果有）
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
