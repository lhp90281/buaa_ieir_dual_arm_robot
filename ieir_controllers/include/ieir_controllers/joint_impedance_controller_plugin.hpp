#ifndef IEIR_CONTROLLERS__JOINT_IMPEDANCE_CONTROLLER_PLUGIN_HPP_
#define IEIR_CONTROLLERS__JOINT_IMPEDANCE_CONTROLLER_PLUGIN_HPP_

#include <memory>
#include <string>
#include <vector>
#include <map>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "sensor_msgs/msg/joint_state.hpp"
#include "ieir_controllers/ieir_dynamics.hpp"

namespace ieir_controllers
{

/**
 * @brief Joint Impedance Controller Plugin for ros2_control
 * 
 * This controller implements impedance control with gravity compensation.
 * Formula: tau = g(q) + Kp*(qd-q) + Kd*(vd-v)
 * 
 * - Reads joint states from state_interfaces
 * - Writes torques to command_interfaces
 * - Receives target commands via topic (with realtime buffer)
 */
class JointImpedanceControllerPlugin : public controller_interface::ControllerInterface
{
public:
  JointImpedanceControllerPlugin();

  /**
   * @brief Get command interface configuration
   */
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  /**
   * @brief Get state interface configuration
   */
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  /**
   * @brief Initialize the controller
   */
  controller_interface::CallbackReturn on_init() override;

  /**
   * @brief Configure the controller
   */
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Activate the controller
   */
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Deactivate the controller
   */
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Update control command (real-time loop)
   */
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Per-joint gains structure
  struct JointGains {
    double stiffness;
    double damping;
    double effort_limit;
  };

  // Target command callback (non-realtime)
  void targetCommandCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

  // Algorithm library
  std::unique_ptr<ieir_dynamics::IeirDynamics> dynamics_;

  // Parameters
  std::vector<std::string> joint_names_;
  std::map<std::string, JointGains> joint_gains_;
  double default_stiffness_ = 20.0;
  double default_damping_ = 5.0;
  double max_effort_ = 50.0;
  double position_error_limit_ = 0.087;  // ~5 degrees
  double velocity_filter_alpha_ = 0.99;
  double velocity_saturation_ = 5.0;

  // State storage
  Eigen::VectorXd q_;
  Eigen::VectorXd v_;
  Eigen::VectorXd v_filtered_;
  
  // Desired state (from target command)
  Eigen::VectorXd q_desired_;
  Eigen::VectorXd v_desired_;
  
  // Gain vectors (aligned with full model velocity indices)
  Eigen::VectorXd stiffness_vec_;
  Eigen::VectorXd damping_vec_;

  // Cached mapping: joint_names_[i] -> velocity index in full Pinocchio model
  std::vector<int> joint_idx_v_;

  // Realtime buffer for target commands
  realtime_tools::RealtimeBuffer<sensor_msgs::msg::JointState::SharedPtr> rt_target_buffer_;
  
  // Subscriber for target commands
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr target_sub_;
  
  // Publisher for feedback (optional, for debugging)
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr feedback_pub_;

  // Flags
  bool target_received_ = false;
  bool initial_pose_captured_ = false;
  bool velocity_initialized_ = false;
  int warmup_counter_ = 0;
  static constexpr int WARMUP_CYCLES = 100;  // 200ms at 500Hz: gravity-only warmup
};

}  // namespace ieir_controllers

#endif  // IEIR_CONTROLLERS__JOINT_IMPEDANCE_CONTROLLER_PLUGIN_HPP_
