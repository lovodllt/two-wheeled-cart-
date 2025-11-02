#ifndef SIMPLE_DIFF_DRIVE_CONTROLLER_HPP_
#define SIMPLE_DIFF_DRIVE_CONTROLLER_HPP_

#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

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

private:
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void compute_wheel_velocities(double linear_x, double angular_z, double & left_vel, double & right_vel);

  // ROS2 相关
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

  // 控制参数
  std::string left_wheel_name_;
  std::string right_wheel_name_;
  double wheel_separation_;
  double wheel_radius_;

  // 状态变量
  double left_wheel_velocity_command_;
  double right_wheel_velocity_command_;
};

#endif  // SIMPLE_DIFF_DRIVE_CONTROLLER_HPP_