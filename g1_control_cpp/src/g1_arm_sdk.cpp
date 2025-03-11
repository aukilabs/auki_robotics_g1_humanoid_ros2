#include "g1_control_cpp/g1_arm_sdk.hpp"
#include "common/motor_crc_hg.h"

#include <string>
#include <vector>
#include <iostream>

namespace g1_control_cpp
{
CallbackReturn G1ArmSDK::on_init(const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  rclcpp::NodeOptions options;
  options.arguments({ "--ros-args", "-r", "__node:=topic_based_ros2_control_" + info_.name });
  node_ = rclcpp::Node::make_shared("_", options);

  logger_ = std::make_shared<rclcpp::Logger>(node_->get_logger());
  clock_ = std::make_shared<rclcpp::Clock>(rclcpp::Clock());
  
  hw_commands_.resize(info_.joints.size(), 0.0);
  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // Each Joints only accept positional command
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has %zu command interfaces found. 1 expected.",
        joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have %s command interfaces found. '%s' expected.",
        joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    // 2 state interfaces is expected, position and velocity
    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }
    // Check if they are in the right position
    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  arm_sdk_publisher_ = node_->create_publisher<unitree_hg::msg::LowCmd>("/arm_sdk",rclcpp::SystemDefaultsQoS());
  motor_state_subscriber_ = node_->create_subscription<unitree_hg::msg::LowState>(
    "/lowstate", rclcpp::SystemDefaultsQoS(), std::bind(&G1ArmSDK::state_callback, this, std::placeholders::_1));

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn G1ArmSDK::on_configure(const rclcpp_lifecycle::State & previous_state)
{
  // Set Weight to zero
  arm_sdk_weight_ = 0.0;
  low_cmd_msg_.motor_cmd.at(JointIndex::kNotUsedJoint).q = arm_sdk_weight_;

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> G1ArmSDK::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> G1ArmSDK::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn G1ArmSDK::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Read State
  RCLCPP_INFO(get_logger(), "Activating... Waiting for Motor State Message.");

  // Configure the low cmd msg
  low_cmd_msg_.mode_pr = mode_pr_;
  low_cmd_msg_.mode_machine = mode_machine_;

  for (int j=0; j<info_.joints.size(); ++j) {
    low_cmd_msg_.motor_cmd.at(arm_joints.at(j)).mode = 1;
    low_cmd_msg_.motor_cmd.at(arm_joints.at(j)).q = 0.0;
    low_cmd_msg_.motor_cmd.at(arm_joints.at(j)).dq = dq_;
    low_cmd_msg_.motor_cmd.at(arm_joints.at(j)).kp = kp_;
    low_cmd_msg_.motor_cmd.at(arm_joints.at(j)).kd = kd_;
    low_cmd_msg_.motor_cmd.at(arm_joints.at(j)).tau = tau_ff_;
    hw_commands_[j] = 0.0;
    RCLCPP_DEBUG(get_logger(), "hw_commands_[%ld]: %f", j, hw_commands_[j]);
  }

  for (int j=1; j<=50; ++j) {
    arm_sdk_weight_ = arm_sdk_weight_ + float(1.0/50.0);
    low_cmd_msg_.motor_cmd.at(JointIndex::kNotUsedJoint).q = arm_sdk_weight_;
    get_crc(low_cmd_msg_);
    arm_sdk_publisher_->publish(low_cmd_msg_);
    RCLCPP_DEBUG(get_logger(), "published weight: %f", arm_sdk_weight_);
    rclcpp::sleep_for(std::chrono::nanoseconds(int(2e8)));
  }

  arm_sdk_weight_ = 1.0f;
  low_cmd_msg_.motor_cmd.at(JointIndex::kNotUsedJoint).q = arm_sdk_weight_;

  RCLCPP_INFO(get_logger(), "Successfully activated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn G1ArmSDK::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (arm_sdk_weight_ == 0) {
    return hardware_interface::CallbackReturn::SUCCESS;
  }
  for (int j=1; j<=50; ++j) {
    arm_sdk_weight_ = arm_sdk_weight_ - float(1.0/50.0);
    low_cmd_msg_.motor_cmd.at(JointIndex::kNotUsedJoint).q = arm_sdk_weight_;
    get_crc(low_cmd_msg_);
    arm_sdk_publisher_->publish(low_cmd_msg_);
    RCLCPP_DEBUG(get_logger(), "published weight: %f", arm_sdk_weight_);
    rclcpp::sleep_for(std::chrono::nanoseconds(int(2e8)));
  }
  RCLCPP_INFO(get_logger(), "Successfully deactivated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

return_type G1ArmSDK::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  if (rclcpp::ok())
  {
    rclcpp::spin_some(node_);
  }

  // Read State
  std::unique_lock<std::mutex> lock(mtx_);
  if(!cv_.wait_for(lock, std::chrono::nanoseconds(1'000'000'000),
    [this] { return !state_msg_queue_.empty(); }))
  {
    RCLCPP_WARN(get_logger(), "/lowstate timed out");
    return return_type::OK;
  }
  
  unitree_hg::msg::LowState msg = state_msg_queue_.front();
  state_msg_queue_.pop();

  // Off set 12 since arm_sdk start at waist 
  for (int j=0u; j<info_.joints.size(); j++) {
    hw_positions_[j] = msg.motor_state.at(arm_joints.at(j)).q;
    hw_velocities_[j] = msg.motor_state.at(arm_joints.at(j)).dq;
    RCLCPP_DEBUG(get_logger(), "joint name: %s    joint index: %d    message index: %d    q: %f",
      info_.joints.at(j).name.c_str(), j, arm_joints.at(j), hw_positions_[j]);
  }
  
  return return_type::OK;
}

double G1ArmSDK::clip_q_target(double target_q, double current_q)
{
  double delta_q = target_q - current_q;
  double motion_scale = abs(delta_q) / arm_velocity_limit_ * control_dt_;
  return current_q + delta_q / std::max(motion_scale, 1.0);
}

return_type G1ArmSDK::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  // Configure the low cmd msg
  low_cmd_msg_.mode_pr = mode_pr_;
  low_cmd_msg_.mode_machine = mode_machine_;

  for (int j=0; j<info_.joints.size(); ++j) {
    // set some default values
    if (std::isnan(hw_positions_[j]) || std::isnan(hw_commands_[j]))
    {
      // skip if NaN value, make sure it can clip
      RCLCPP_DEBUG(get_logger(), "hw_commands_ %f", hw_commands_[j]);
      continue;
    }

    double target_q = clip_q_target(hw_commands_[j], hw_positions_[j]);
    low_cmd_msg_.motor_cmd.at(arm_joints.at(j)).q = target_q;
    low_cmd_msg_.motor_cmd.at(arm_joints.at(j)).dq = dq_;
    low_cmd_msg_.motor_cmd.at(arm_joints.at(j)).kp = kp_;
    low_cmd_msg_.motor_cmd.at(arm_joints.at(j)).kd = kd_;
    low_cmd_msg_.motor_cmd.at(arm_joints.at(j)).tau = tau_ff_;

    RCLCPP_INFO(get_logger(), "Writing: joint name: %s    motor index: %d    hw_command: %f    hw_position: %f    target_q: %f", 
    info_.joints.at(j).name.c_str(), arm_joints.at(j), hw_commands_[j],  hw_positions_[j], target_q);
  }
  low_cmd_msg_.motor_cmd.at(JointIndex::kNotUsedJoint).q = arm_sdk_weight_;

  get_crc(low_cmd_msg_);

  // Send Command
  arm_sdk_publisher_->publish(low_cmd_msg_);

  return return_type::OK;
}

void G1ArmSDK::state_callback(const unitree_hg::msg::LowState::SharedPtr msg)
{
  RCLCPP_DEBUG(node_->get_logger(), "got message in state_callback");
  std::lock_guard<std::mutex> lock(mtx_);
  state_msg_queue_.push(*msg);
  cv_.notify_all();  
}

}  // namespace ros2_control_demo_example_7

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  g1_control_cpp::G1ArmSDK, hardware_interface::SystemInterface)
