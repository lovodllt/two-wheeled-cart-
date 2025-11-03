#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class TimedVelocityPlanner(Node):
    def __init__(self):
        super().__init__("timed_velocity_planner")

        # 1. 配置参数（可直接修改）
        self.robot_namespace = "robot1"  # 小车命名空间
        self.start_delay = 5.0  # 启动后延迟5秒开始执行（等待控制器初始化）

        # 2. 定义时序速度指令：[(开始时间s, 持续时间s, 线速度m/s, 角速度rad/s), ...]
        # 说明：
        # - 开始时间：相对于脚本启动的时间（含start_delay）
        # - 持续时间：该速度命令执行的时长
        # - 线速度：x方向速度（前进/后退），角速度：z方向（左转/右转）
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

        # 3. 初始化变量
        self.start_time = time.time()
        self.cmd_vel_pub = None
        self.timer = None

        # 4. 延迟启动定时器（等待小车控制器初始化）
        self.get_logger().info(f"时序速度规划器启动，{self.start_delay}秒后开始执行命令...")
        self.delay_timer = self.create_timer(self.start_delay, self.start_execution)

    def start_execution(self):
        """延迟后开始执行速度命令"""
        self.delay_timer.cancel()  # 关闭延迟定时器

        # 创建速度发布器
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            f"/{self.robot_namespace}/cmd_vel",
            10
        )

        # 10Hz定时器：实时检查并发布当前时间段的速度命令
        self.timer = self.create_timer(0.1, self.execute_timed_commands)
        self.get_logger().info("开始执行时序速度命令！")
        self.get_logger().info(f"命令列表：{self.timed_commands}")

    def execute_timed_commands(self):
        """执行当前时间段对应的速度命令"""
        current_time = time.time() - self.start_time  # 脚本启动后的总耗时
        current_cmd = None

        # 遍历所有命令，找到当前时间对应的命令
        for (start_t, duration_t, linear_x, angular_z) in self.timed_commands:
            end_t = start_t + duration_t
            if start_t <= current_time < end_t:
                current_cmd = (linear_x, angular_z)
                break

        # 发布当前命令（无匹配命令则停止）
        cmd_msg = Twist()
        if current_cmd:
            cmd_msg.linear.x = current_cmd[0]
            cmd_msg.angular.z = current_cmd[1]
            self.get_logger().debug(
                f"当前时间：{current_time:.1f}s | 线速度：{current_cmd[0]:.2f} | 角速度：{current_cmd[1]:.2f}"
            )
        else:
            # 所有命令执行完毕，停止运动
            cmd_msg.linear.x = 0.0
            cmd_msg.angular.z = 0.0
            self.get_logger().info("所有时序命令执行完毕！小车停止运动")
            self.timer.cancel()  # 关闭定时器

        self.cmd_vel_pub.publish(cmd_msg)

def main(args=None):
    rclpy.init(args=args)
    planner = TimedVelocityPlanner()
    rclpy.spin(planner)
    planner.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
