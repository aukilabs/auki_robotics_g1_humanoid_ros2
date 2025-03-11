#ifndef G1_CONTROL_CPP__G1_ARM_SDK_HPP_
#define G1_CONTROL_CPP__G1_ARM_SDK_HPP_

#include "string"
#include "unordered_map"
#include "vector"
#include <queue>
#include <mutex>
#include <condition_variable>
#include <optional>

#include "rclcpp/version.h"
#include "rclcpp/node.hpp"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "unitree_hg/msg/low_state.hpp"
#include "unitree_hg/msg/low_cmd.hpp"

using hardware_interface::return_type;

namespace g1_control_cpp
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

enum JointIndex {
  // Left leg
  kLeftHipPitch,
  kLeftHipRoll,
  kLeftHipYaw,
  kLeftKnee,
  kLeftAnkle,
  kLeftAnkleRoll,

  // Right leg
  kRightHipPitch,
  kRightHipRoll,
  kRightHipYaw,
  kRightKnee,
  kRightAnkle,
  kRightAnkleRoll,

  kWaistYaw,
  kWaistRoll,
  kWaistPitch,

  // Left arm
  kLeftShoulderPitch,
  kLeftShoulderRoll,
  kLeftShoulderYaw,
  kLeftElbow,
  kLeftWistRoll,
  kLeftWistPitch,
  kLeftWistYaw,
  // Right arm
  kRightShoulderPitch,
  kRightShoulderRoll,
  kRightShoulderYaw,
  kRightElbow,
  kRightWistRoll,
  kRightWistPitch,
  kRightWistYaw,

  kNotUsedJoint,
  kNotUsedJoint1,
  kNotUsedJoint2,
  kNotUsedJoint3,
  kNotUsedJoint4,
  kNotUsedJoint5
};

std::array<JointIndex, 17> arm_joints = {
  JointIndex::kWaistYaw,
  JointIndex::kWaistRoll,
  JointIndex::kWaistPitch,
  JointIndex::kLeftShoulderPitch,  JointIndex::kLeftShoulderRoll,
  JointIndex::kLeftShoulderYaw,    JointIndex::kLeftElbow,
  JointIndex::kLeftWistRoll,       JointIndex::kLeftWistPitch,
  JointIndex::kLeftWistYaw,
  JointIndex::kRightShoulderPitch, JointIndex::kRightShoulderRoll,
  JointIndex::kRightShoulderYaw,   JointIndex::kRightElbow,
  JointIndex::kRightWistRoll,      JointIndex::kRightWistPitch,
  JointIndex::kRightWistYaw
};

class HARDWARE_INTERFACE_PUBLIC G1ArmSDK : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  /// Get the logger of the SystemInterface.
  /**
   * \return logger of the SystemInterface.
   */
  rclcpp::Logger get_logger() const { return *logger_; }

  /// Get the clock of the SystemInterface.
  /**
   * \return clock of the SystemInterface.
   */
  rclcpp::Clock::SharedPtr get_clock() const { return clock_; }

  void state_callback(const unitree_hg::msg::LowState::SharedPtr msg);
  
  double clip_q_target(double target_q, double current_q);

protected:
  // Objects for logging
  std::shared_ptr<rclcpp::Logger> logger_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Node::SharedPtr node_;

  // States
  unitree_hg::msg::LowCmd low_cmd_msg_;
  uint8_t mode_pr_ = 255;
  uint8_t mode_machine_ = 255;
  float arm_sdk_weight_ = 0.0f;
  float weight_rate_ = 0.2f;
  double arm_velocity_limit_ = 20.0;
  double control_dt_ = 1.0 / 250.0;
  float kp_ = 60.f;
  float kd_ = 1.5f;
  float dq_ = 0.f;
  float tau_ff_ = 0.f;


  // Store the command
  std::vector<double> hw_commands_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;

  // Store the state messages
  std::queue<unitree_hg::msg::LowState> state_msg_queue_;
  mutable std::mutex mtx_;
  std::condition_variable cv_;

  std::shared_ptr<rclcpp::Publisher<unitree_hg::msg::LowCmd>> arm_sdk_publisher_;
  std::shared_ptr<rclcpp::Subscription<unitree_hg::msg::LowState>> motor_state_subscriber_;
};

}  // namespace g1_control_cpp

#endif  // G1_CONTROL_CPP__G1_ARM_SDK_HPP_
