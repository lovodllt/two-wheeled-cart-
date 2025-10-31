#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64.hpp"
#include <termios.h>
#include <unistd.h>
#include <csignal>

struct termios original_termios;

void reset_terminal() {
    tcsetattr(STDIN_FILENO, TCSANOW, &original_termios);
}

void set_terminal_raw() {
    tcgetattr(STDIN_FILENO, &original_termios);
    atexit(reset_terminal);

    struct termios raw = original_termios;
    raw.c_lflag &= ~(ECHO | ICANON);
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);
}

char get_key() {
    char c;
    if (read(STDIN_FILENO, &c, 1) < 1) {
        return 0;
    }
    return c;
}

class SimpleDiffDrive : public rclcpp::Node {
public:
    SimpleDiffDrive() : Node("simple_diff_drive") {
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10
        );

        print_control_guide();

        RCLCPP_INFO(this->get_logger(), "差分驱动控制器已启动，发布到话题: /cmd_vel");
    }

    void publish_cmd_vel(double linear_x, double angular_z) {
        auto twist_msg = geometry_msgs::msg::Twist();
        twist_msg.linear.x = linear_x;
        twist_msg.angular.z = angular_z;
        cmd_vel_pub_->publish(twist_msg);

        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            500,
            "速度指令 | 线速度: %.2f m/s | 角速度: %.2f rad/s",
            linear_x, angular_z
        );
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

    void print_control_guide() {
        RCLCPP_INFO(this->get_logger(), "===== Gazebo 两轮机器人键盘控制器 =====");
        RCLCPP_INFO(this->get_logger(), "w : 前进 (0.5 m/s)");
        RCLCPP_INFO(this->get_logger(), "s : 后退 (-0.5 m/s)");
        RCLCPP_INFO(this->get_logger(), "a : 左转向 (0.5 rad/s)");
        RCLCPP_INFO(this->get_logger(), "d : 右转向 (-0.5 rad/s)");
        RCLCPP_INFO(this->get_logger(), "q : 前进 + 左转向 (0.3 + 0.3)");
        RCLCPP_INFO(this->get_logger(), "e : 前进 + 右转向 (0.3 - 0.3)");
        RCLCPP_INFO(this->get_logger(), "z : 后退 + 左转向 (-0.3 + 0.3)");
        RCLCPP_INFO(this->get_logger(), "c : 后退 + 右转向 (-0.3 - 0.3)");
        RCLCPP_INFO(this->get_logger(), "x : 紧急停止");
        RCLCPP_INFO(this->get_logger(), "CTRL+C : 退出控制器");
        RCLCPP_INFO(this->get_logger(), "======================================");
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto controller_node = std::make_shared<SimpleDiffDrive>();

    // 配置终端为非阻塞模式
    set_terminal_raw();

    RCLCPP_INFO(controller_node->get_logger(), "键盘控制器就绪，开始监听按键输入...");

    // 主控制循环
    while (rclcpp::ok()) {
        char key = get_key();
        switch (key) {
            case 'w': controller_node->publish_cmd_vel(0.5, 0.0);  break;   // 前进
            case 's': controller_node->publish_cmd_vel(-0.5, 0.0); break;   // 后退
            case 'a': controller_node->publish_cmd_vel(0.0, 0.5);  break;   // 左转
            case 'd': controller_node->publish_cmd_vel(0.0, -0.5); break;   // 右转
            case 'q': controller_node->publish_cmd_vel(0.3, 0.3);  break;   // 前进+左转
            case 'e': controller_node->publish_cmd_vel(0.3, -0.3); break;   // 前进+右转
            case 'z': controller_node->publish_cmd_vel(-0.3, 0.3); break;   // 后退+左转
            case 'c': controller_node->publish_cmd_vel(-0.3, -0.3);break;   // 后退+右转
            case 'x': controller_node->publish_cmd_vel(0.0, 0.0);  break;   // 停止
            case 3:   // CTRL+C
                RCLCPP_INFO(controller_node->get_logger(), "收到退出信号，停止机器人...");
                controller_node->publish_cmd_vel(0.0, 0.0);
                rclcpp::shutdown();
                return 0;
            default:
                if (key != 0) {
                    RCLCPP_WARN_THROTTLE(
                        controller_node->get_logger(),
                        *controller_node->get_clock(),
                        1000,
                        "未知按键: %c (按 w/a/s/d/x 控制机器人)", key
                    );
                }
                break;
        }

        rclcpp::spin_some(controller_node);
        usleep(50000); // 50ms 延迟
    }

    // 退出清理
    controller_node->publish_cmd_vel(0.0, 0.0);
    rclcpp::shutdown();
    return 0;
}