#ifndef SIMPLE_DIFF_DRIVE_CONTROLLER__SIMPLE_DIFF_DRIVE_CONTROLLER_HPP_
#define SIMPLE_DIFF_DRIVE_CONTROLLER__SIMPLE_DIFF_DRIVE_CONTROLLER_HPP_

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <memory>
#include <string>

class SimpleDiffDriveController : public controller_interface::ControllerInterface
{
public:
  SimpleDiffDriveController();

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  // 四个车轮的名称（成员变量新增）
  std::string front_left_wheel_name_;
  std::string front_right_wheel_name_;
  std::string rear_left_wheel_name_;
  std::string rear_right_wheel_name_;

  // 车辆参数（新增wheelbase）
  double wheel_separation_;  // 左右轮间距
  double wheelbase_;         // 前后轮距
  double wheel_radius_;      // 车轮半径
  double max_velocity_;      // 最大车轮速度

  // 车轮速度命令
  double left_wheel_velocity_command_;  // 左侧车轮（前+后）速度
  double right_wheel_velocity_command_; // 右侧车轮（前+后）速度

  // 速度命令订阅者
  std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::Twist>> cmd_vel_sub_;

  // 回调函数和工具函数
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void compute_wheel_velocities(double linear_x, double angular_z,
                                double & left_vel, double & right_vel);
};

#endif  // SIMPLE_DIFF_DRIVE_CONTROLLER__SIMPLE_DIFF_DRIVE_CONTROLLER_HPP_
