#include "right_wheel_controller.hpp"
#include <controller_interface/controller_interface.hpp>

RightWheelController::RightWheelController()
: velocity_command_(0.0)
{
}

controller_interface::InterfaceConfiguration RightWheelController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.push_back(wheel_name_ + "/" + hardware_interface::HW_IF_VELOCITY);
  return config;
}

controller_interface::InterfaceConfiguration RightWheelController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.push_back(wheel_name_ + "/" + hardware_interface::HW_IF_POSITION);
  config.names.push_back(wheel_name_ + "/" + hardware_interface::HW_IF_VELOCITY);
  return config;
}

controller_interface::CallbackReturn RightWheelController::on_init()
{
  try
  {
    // 声明参数
    auto node = this->get_node();
    node->declare_parameter("wheel_name", "right_wheel_joint");
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception thrown during init: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RightWheelController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto node = this->get_node();
  
  // 获取参数
  wheel_name_ = node->get_parameter("wheel_name").as_string();
  
  RCLCPP_INFO(node->get_logger(), "Configuring right wheel controller for: %s", wheel_name_.c_str());
  
  // 创建订阅者
  velocity_command_sub_ = node->create_subscription<std_msgs::msg::Float64>(
    "~/command", 10,
    std::bind(&RightWheelController::velocity_command_callback, this, std::placeholders::_1));
  
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RightWheelController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // 重置命令
  velocity_command_ = 0.0;
  
  RCLCPP_INFO(get_node()->get_logger(), "Right wheel controller activated");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RightWheelController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // 停止轮子
  velocity_command_ = 0.0;
  
  RCLCPP_INFO(get_node()->get_logger(), "Right wheel controller deactivated");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type RightWheelController::update(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // 使用关节句柄设置轮子速度命令
  for (auto & command_interface : command_interfaces_)
  {
    if (command_interface.get_prefix_name() == wheel_name_ && 
        command_interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY)
    {
      command_interface.set_value(velocity_command_);
      RCLCPP_DEBUG(get_node()->get_logger(), "Setting %s velocity to: %.2f", wheel_name_.c_str(), velocity_command_);
    }
  }
  
  // 读取并打印状态信息
  double position = 0.0;
  double velocity = 0.0;
  
  for (const auto & state_interface : state_interfaces_)
  {
    if (state_interface.get_prefix_name() == wheel_name_)
    {
      if (state_interface.get_interface_name() == hardware_interface::HW_IF_POSITION)
      {
        position = state_interface.get_value();
      }
      else if (state_interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY)
      {
        velocity = state_interface.get_value();
      }
    }
  }
  
  static int count = 0;
  if (count++ % 100 == 0)  // 每100次更新打印一次
  {
    RCLCPP_INFO(get_node()->get_logger(), 
                "Wheel %s - Command: %.2f, Position: %.2f, Velocity: %.2f", 
                wheel_name_.c_str(), velocity_command_, position, velocity);
  }
  
  return controller_interface::return_type::OK;
}

void RightWheelController::velocity_command_callback(const std_msgs::msg::Float64::SharedPtr msg)
{
  velocity_command_ = msg->data;
  RCLCPP_INFO(get_node()->get_logger(), "Received velocity command: %.2f", velocity_command_);
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(RightWheelController, controller_interface::ControllerInterface)