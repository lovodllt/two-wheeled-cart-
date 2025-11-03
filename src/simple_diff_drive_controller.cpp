#include "simple_diff_drive_controller.hpp"
#include <controller_interface/controller_interface.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <vector>

SimpleDiffDriveController::SimpleDiffDriveController()
: left_wheel_velocity_command_(0.0),
  right_wheel_velocity_command_(0.0)
{
}

// 声明4个车轮的命令接口（速度控制）
controller_interface::InterfaceConfiguration SimpleDiffDriveController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // 四个车轮的速度控制接口（与URDF关节名称完全一致）
  config.names.push_back(front_left_wheel_name_ + "/" + hardware_interface::HW_IF_VELOCITY);
  config.names.push_back(front_right_wheel_name_ + "/" + hardware_interface::HW_IF_VELOCITY);
  config.names.push_back(rear_left_wheel_name_ + "/" + hardware_interface::HW_IF_VELOCITY);
  config.names.push_back(rear_right_wheel_name_ + "/" + hardware_interface::HW_IF_VELOCITY);

  return config;
}

// 声明4个车轮的状态接口（位置读取）
controller_interface::InterfaceConfiguration SimpleDiffDriveController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // 四个车轮的位置状态接口
  config.names.push_back(front_left_wheel_name_ + "/" + hardware_interface::HW_IF_POSITION);
  config.names.push_back(front_right_wheel_name_ + "/" + hardware_interface::HW_IF_POSITION);
  config.names.push_back(rear_left_wheel_name_ + "/" + hardware_interface::HW_IF_POSITION);
  config.names.push_back(rear_right_wheel_name_ + "/" + hardware_interface::HW_IF_POSITION);

  return config;
}

// 初始化：声明4个车轮的参数
controller_interface::CallbackReturn SimpleDiffDriveController::on_init()
{
  try
  {
    auto node = this->get_node();

    // 声明四个车轮的名称参数（默认值与URDF一致）
    node->declare_parameter("front_left_wheel_name", "front_left_wheel_joint");
    node->declare_parameter("front_right_wheel_name", "front_right_wheel_joint");
    node->declare_parameter("rear_left_wheel_name", "rear_left_wheel_joint");
    node->declare_parameter("rear_right_wheel_name", "rear_right_wheel_joint");

    // 车辆核心参数
    node->declare_parameter("wheel_separation", 0.3);  // 左右轮间距（与URDF一致）
    node->declare_parameter("wheelbase", 0.4);         // 前后轮距（与URDF一致）
    node->declare_parameter("wheel_radius", 0.1);      // 车轮半径（与URDF一致）
    node->declare_parameter("max_velocity", 10.0);     // 最大车轮速度
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception thrown during init: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

// 配置：读取参数并创建订阅者
controller_interface::CallbackReturn SimpleDiffDriveController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto node = this->get_node();


  // 读取车辆参数
  wheel_separation_ = node->get_parameter("wheel_separation").as_double();
  wheelbase_ = node->get_parameter("wheelbase").as_double();
  wheel_radius_ = node->get_parameter("wheel_radius").as_double();
  max_velocity_ = node->get_parameter("max_velocity").as_double();


  // 创建速度命令订阅者（全局话题 /cmd_vel，方便控制）
  cmd_vel_sub_ = node->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10,  // 话题名称：/cmd_vel（全局，无需加命名空间）
    std::bind(&SimpleDiffDriveController::cmd_vel_callback, this, std::placeholders::_1));

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SimpleDiffDriveController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // 激活时重置车轮速度命令
  left_wheel_velocity_command_ = 0.0;
  right_wheel_velocity_command_ = 0.0;

  RCLCPP_INFO(get_node()->get_logger(), "Four-wheel diff drive controller activated");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SimpleDiffDriveController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // 停用时停止所有车轮
  left_wheel_velocity_command_ = 0.0;
  right_wheel_velocity_command_ = 0.0;

  RCLCPP_INFO(get_node()->get_logger(), "Four-wheel diff drive controller deactivated");
  return controller_interface::CallbackReturn::SUCCESS;
}

// 更新：将计算出的速度命令下发到4个车轮
controller_interface::return_type SimpleDiffDriveController::update(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // 为4个车轮设置速度命令（同侧车轮速度相同）
  for (auto & command_interface : command_interfaces_)
  {
    // 前轮左侧 + 后轮左侧：速度相同
    if ((command_interface.get_prefix_name() == front_left_wheel_name_ ||
         command_interface.get_prefix_name() == rear_left_wheel_name_) &&
        command_interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY)
    {
      command_interface.set_value(left_wheel_velocity_command_);
    }
    // 前轮右侧 + 后轮右侧：速度相同
    else if ((command_interface.get_prefix_name() == front_right_wheel_name_ ||
              command_interface.get_prefix_name() == rear_right_wheel_name_) &&
             command_interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY)
    {
      command_interface.set_value(right_wheel_velocity_command_);
    }
  }

  return controller_interface::return_type::OK;
}

// 速度命令回调：接收/cmd_vel，计算车轮速度
void SimpleDiffDriveController::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  // 计算左右侧车轮的目标速度（同侧前后轮速度一致）
  compute_wheel_velocities(
    msg->linear.x,  // 线速度（前进/后退）
    msg->angular.z, // 角速度（转向）
    left_wheel_velocity_command_,  // 左侧车轮速度（前+后）
    right_wheel_velocity_command_  // 右侧车轮速度（前+后）
  );
}

// 四轮差速运动学模型：计算左右侧车轮速度
void SimpleDiffDriveController::compute_wheel_velocities(double linear_x, double angular_z,
                                                        double & left_vel, double & right_vel)
{
  // 核心公式（基于差速驱动模型，同侧前后轮同步）：
  // 左侧车轮速度 = (线速度 - 角速度 × 轮距/2) / 车轮半径
  // 右侧车轮速度 = (线速度 + 角速度 × 轮距/2) / 车轮半径
  left_vel = (linear_x - angular_z * wheel_separation_ / 2.0) / wheel_radius_;
  right_vel = (linear_x + angular_z * wheel_separation_ / 2.0) / wheel_radius_;

  // 限制最大速度（避免车轮超速）
  left_vel = std::clamp(left_vel, -max_velocity_, max_velocity_);
  right_vel = std::clamp(right_vel, -max_velocity_, max_velocity_);

  RCLCPP_DEBUG(get_node()->get_logger(), "Computed wheel velocities: left=%.3f, right=%.3f", left_vel, right_vel);
}

// 插件注册（保持不变）
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(SimpleDiffDriveController, controller_interface::ControllerInterface)
