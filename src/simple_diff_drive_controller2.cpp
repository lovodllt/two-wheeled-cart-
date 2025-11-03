#include "simple_diff_drive_controller2.hpp"
#include <controller_interface/controller_interface.hpp>
#include <geometry_msgs/msg/twist.hpp>

SimpleDiffDriveController2::SimpleDiffDriveController2()
: left_wheel_velocity_command_(0.0),
  right_wheel_velocity_command_(0.0)
{
}

controller_interface::InterfaceConfiguration SimpleDiffDriveController2::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  // 声明需要控制的命令接口：左右轮的速度控制
  config.names.push_back(left_wheel_name_ + "/" + hardware_interface::HW_IF_VELOCITY);
  config.names.push_back(right_wheel_name_ + "/" + hardware_interface::HW_IF_VELOCITY);
  return config;
}

controller_interface::InterfaceConfiguration SimpleDiffDriveController2::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  // 声明需要读取的状态接口：左右轮的位置
  config.names.push_back(left_wheel_name_ + "/" + hardware_interface::HW_IF_POSITION);
  config.names.push_back(right_wheel_name_ + "/" + hardware_interface::HW_IF_POSITION);
  return config;
}

controller_interface::CallbackReturn SimpleDiffDriveController2::on_init()
{
  try
  {
    // 声明控制器参数
    auto node = this->get_node();
    node->declare_parameter("left_wheel_name", "left_wheel_joint");
    node->declare_parameter("right_wheel_name", "right_wheel_joint");
    node->declare_parameter("wheel_separation", 0.3);  // 轮子间距
    node->declare_parameter("wheel_radius", 0.1);      // 轮子半径
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception thrown during init: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SimpleDiffDriveController2::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto node = this->get_node();

  // 从参数服务器获取参数值
  left_wheel_name_ = node->get_parameter("left_wheel_name").as_string();
  right_wheel_name_ = node->get_parameter("right_wheel_name").as_string();
  wheel_separation_ = node->get_parameter("wheel_separation").as_double();
  wheel_radius_ = node->get_parameter("wheel_radius").as_double();

  RCLCPP_INFO(node->get_logger(), "Configuring simple diff drive controller2:");
  RCLCPP_INFO(node->get_logger(), "  Left wheel: %s", left_wheel_name_.c_str());
  RCLCPP_INFO(node->get_logger(), "  Right wheel: %s", right_wheel_name_.c_str());
  RCLCPP_INFO(node->get_logger(), "  Wheel separation: %.3f", wheel_separation_);
  RCLCPP_INFO(node->get_logger(), "  Wheel radius: %.3f", wheel_radius_);

  // 创建速度命令订阅者 - 使用标准的cmd_vel话题
  cmd_vel_sub_ = node->create_subscription<geometry_msgs::msg::Twist>(
    "~/cmd_vel", 10,  // 话题名称：/simple_diff_drive_controller2/cmd_vel
    std::bind(&SimpleDiffDriveController2::cmd_vel_callback, this, std::placeholders::_1));

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SimpleDiffDriveController2::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // 重置命令
  left_wheel_velocity_command_ = 0.0;
  right_wheel_velocity_command_ = 0.0;

  RCLCPP_INFO(get_node()->get_logger(), "Simple diff drive controller2 activated");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SimpleDiffDriveController2::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // 停止轮子
  left_wheel_velocity_command_ = 0.0;
  right_wheel_velocity_command_ = 0.0;

  RCLCPP_INFO(get_node()->get_logger(), "Simple diff drive controller2 deactivated");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type SimpleDiffDriveController2::update(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // 设置左右轮的速度命令
  for (auto & command_interface : command_interfaces_)
  {
    if (command_interface.get_prefix_name() == left_wheel_name_ &&
        command_interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY)
    {
      command_interface.set_value(left_wheel_velocity_command_);
    }
    else if (command_interface.get_prefix_name() == right_wheel_name_ &&
             command_interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY)
    {
      command_interface.set_value(right_wheel_velocity_command_);
    }
  }

  return controller_interface::return_type::OK;
}

void SimpleDiffDriveController2::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  // 当收到速度命令时，计算左右轮需要的速度
  compute_wheel_velocities(msg->linear.x, msg->angular.z,
                          left_wheel_velocity_command_, right_wheel_velocity_command_);
}

void SimpleDiffDriveController2::compute_wheel_velocities(double linear_x, double angular_z,
                                                        double & left_vel, double & right_vel)
{
  // 差速运动学模型：
  // 左轮速度 = (线速度 - 角速度 × 轮距/2) / 轮子半径
  // 右轮速度 = (线速度 + 角速度 × 轮距/2) / 轮子半径
  left_vel = (linear_x - angular_z * wheel_separation_ / 2.0) / wheel_radius_;
  right_vel = (linear_x + angular_z * wheel_separation_ / 2.0) / wheel_radius_;

  // 限制最大速度
  const double max_velocity = 10.0;
  left_vel = std::max(std::min(left_vel, max_velocity), -max_velocity);
  right_vel = std::max(std::min(right_vel, max_velocity), -max_velocity);
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(SimpleDiffDriveController2, controller_interface::ControllerInterface)