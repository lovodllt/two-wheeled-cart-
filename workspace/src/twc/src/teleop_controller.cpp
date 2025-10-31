#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
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

class GazeboTeleopController : public rclcpp::Node {
public:
    GazeboTeleopController() : Node("gazebo_teleop_controller") {

        this->declare_parameter("cmd_vel_topic", "/diff_drive_controller/cmd_vel_unstamped");

        std::string cmd_vel_topic;
        this->get_parameter("cmd_vel_topic", cmd_vel_topic);

        vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            cmd_vel_topic,
            10
        );

        RCLCPP_INFO(this->get_logger(), "控制器已启动，发布到话题: %s", cmd_vel_topic.c_str());
        print_control_guide();
    }


    void publish_velocity(double linear_x, double angular_z) {
        auto twist_msg = geometry_msgs::msg::Twist();
        twist_msg.linear.x = linear_x;
        twist_msg.angular.z = angular_z;
        vel_pub_->publish(twist_msg);


        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            500,
            "Gazebo 速度指令 | 线速度: %.2fm/s | 角速度: %.2frad/s",
            linear_x, angular_z
        );
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;


    void print_control_guide() {
        RCLCPP_INFO(this->get_logger(), "===== Gazebo 两轮机器人键盘控制器 =====");
        RCLCPP_INFO(this->get_logger(), "w : 前进 (0.5m/s)");
        RCLCPP_INFO(this->get_logger(), "s : 后退 (-0.5m/s)");
        RCLCPP_INFO(this->get_logger(), "a : 左转向 (1.0rad/s)");
        RCLCPP_INFO(this->get_logger(), "d : 右转向 (-1.0rad/s)");
        RCLCPP_INFO(this->get_logger(), "q : 前进 + 左转向");
        RCLCPP_INFO(this->get_logger(), "e : 前进 + 右转向");
        RCLCPP_INFO(this->get_logger(), "z : 后退 + 左转向");
        RCLCPP_INFO(this->get_logger(), "c : 后退 + 右转向");
        RCLCPP_INFO(this->get_logger(), "x : 紧急停止");
        RCLCPP_INFO(this->get_logger(), "CTRL+C : 退出控制器");
        RCLCPP_INFO(this->get_logger(), "======================================");
    }
};

int main(int argc, char *argv[]) {

    rclcpp::init(argc, argv);
    auto controller_node = std::make_shared<GazeboTeleopController>();


    set_terminal_raw();

    RCLCPP_INFO(controller_node->get_logger(), "键盘控制器就绪，开始监听按键输入...");


    while (rclcpp::ok()) {
        char key = get_key();
        switch (key) {

            case 'w': controller_node->publish_velocity(0.5, 0.0);  break; // 前进
            case 's': controller_node->publish_velocity(-0.5, 0.0); break; // 后退
            case 'a': controller_node->publish_velocity(0.0, 1.0);  break; // 左转
            case 'd': controller_node->publish_velocity(0.0, -1.0); break; // 右转
            case 'q': controller_node->publish_velocity(0.5, 1.0);  break; // 前进+左转
            case 'e': controller_node->publish_velocity(0.5, -1.0); break; // 前进+右转
            case 'z': controller_node->publish_velocity(-0.5, 1.0); break; // 后退+左转
            case 'c': controller_node->publish_velocity(-0.5, -1.0);break; // 后退+右转
            case 'x': controller_node->publish_velocity(0.0, 0.0);  break; // 停止
            case 3:
                RCLCPP_INFO(controller_node->get_logger(), "收到退出信号，停止机器人...");
                controller_node->publish_velocity(0.0, 0.0);
                rclcpp::shutdown();
                return 0;
            default:
                if (key != 0) {
                    RCLCPP_WARN(controller_node->get_logger(), "未知按键: %c", key);
                }
                break;
        }

        rclcpp::spin_some(controller_node);
        usleep(50000);
    }

    controller_node->publish_velocity(0.0, 0.0);
    rclcpp::shutdown();
    return 0;
}