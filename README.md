# two-wheeled-cart-
室内双轮小车
需要自己新建文件夹作为工作空间，

    mkdir ws
    cd ws
    mdkir src
    cd src
    git clone https://github.com/LLLZYC/two-wheeled-cart-.git
    cd ..
    
    rosdep install --from-paths src --ignore-src -r -y
    //安装依赖

    colcon build
    source install/setup.bash
    ros2 launch two_wheeled_cart test.launch.py
    
    sudo apt install ros-humble-teleop-twist-keyboard
    // 安装键盘控制包
    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/robot2/simple_diff_drive_controller/cmd_vel
    //使用这个可以指定键盘控制话题发布，使用键盘控制机器人（另开终端
    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/robot2/simple_diff_drive_controller2/cmd_vel
    //控制二号机器人
    
    