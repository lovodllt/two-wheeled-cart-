#ifndef RIGHT_WHEEL_CONTROLLER_HPP_
#define RIGHT_WHEEL_CONTROLLER_HPP_

#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

class RightWheelController : public controller_interface::ControllerInterface
{
public:
    RightWheelController();

    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    controller_interface::CallbackReturn on_init() override;
    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
    controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
    controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    void velocity_command_callback(const std_msgs::msg::Float64::SharedPtr msg);

    // ROS2 相关
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr velocity_command_sub_;

    // 控制参数
    std::string wheel_name_;
    double velocity_command_;
};

#endif  // RIGHT_WHEEL_CONTROLLER_HPP_