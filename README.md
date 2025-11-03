将pkg包放进工作空间的src后
    

    rosdep install --from-paths src --ignore-src -r -y
    //安装依赖
    colcon build
    source install/setup.bash
    ros2 launch two_wheeled_cart robot1.launch.py
    //可以加载出世界，还有机器人
    
三台不一样的机器人

    ros2 launch two_wheeled_cart color.launch.py
    //颜色不一样
    ros2 launch two_wheeled_cart robot3.launch.py
    //大小不一样，并且有云台，有单独的参数文件

键盘控制

    sudo apt install ros-humble-teleop-twist-keyboard
    // 安装键盘控制包，再开启一个终端可以控制机器人
    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/robot1/cmd_vel
    

定点巡航

    ros2 launch two_wheeled_cart traject.launch.py

修改目的地点在 scripts 文件下面的py文件中

            self.timed_commands = [
            (5.0, 2.0, 0.3, 0.0),    # 5秒开始，持续2秒：前进（0.3m/s）
            (7.0, 1.5, 0.0, 0.8),    # 7秒开始，持续1.5秒：左转（0.8rad/s，约45度）
            (8.5, 2.0, 0.3, 0.0),    # 8.5秒开始，持续2秒：前进（0.3m/s）
            (10.5, 1.5, 0.0, 0.8),   # 10.5秒开始，持续1.5秒：左转
            (12.0, 2.0, 0.3, 0.0),   # 12秒开始，持续2秒：前进
            (14.0, 1.5, 0.0, 0.8),   # 14秒开始，持续1.5秒：左转
            (15.5, 2.0, 0.3, 0.0),   # 15.5秒开始，持续2秒：前进（回到起点附近）
            (17.5, 2.0, 0.0, 0.0)    # 17.5秒开始，持续2秒：停止运动
        ]
    
着一个部分（有注释