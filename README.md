# two-wheeled-cart-
室内双轮小车


    rosdep install --from-paths src --ignore-src -r -y
    //安装依赖

    colcon build
    source install/setup.bash
    ros2 launch two_wheeled_cart world.launch.py