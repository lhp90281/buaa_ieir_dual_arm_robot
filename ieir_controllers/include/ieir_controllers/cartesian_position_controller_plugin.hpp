#ifndef IEIR_CONTROLLERS__CARTESIAN_POSITION_CONTROLLER_PLUGIN_HPP_
#define IEIR_CONTROLLERS__CARTESIAN_POSITION_CONTROLLER_PLUGIN_HPP_

#include <atomic>
#include <memory>
#include <string>
#include <vector>
#include <map>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"
#include "ieir_controllers/ieir_dynamics.hpp"

namespace ieir_controllers
{

/**
 * @brief Cartesian Position Controller for DM motors in MIT mode.
 *
 * Receives target end-effector poses, solves Pinocchio damped-LS IK to
 * obtain joint angles, smooths them under a max joint-speed limit, and
 * forwards the (pos, vel_ff, kp, kd) tuple to the per-joint MIT-mode
 * command interfaces. The DM motor itself runs the PD inner loop:
 *     tau_motor = kp*(q_des - q) + kd*(qdot_des - qdot)
 *               + tau_gravity (from gravity_compensation_controller)
 * so this controller never writes torque directly -- exactly matching
 * the joint_position_controller interface contract, just with the
 * setpoint coming from Cartesian-space IK instead of a JointTrajectory.
 *
 * Designed to coexist with gravity_compensation_controller (which owns
 * the effort interface). Mutually exclusive with joint_position_controller
 * (also writes the position / stiffness / damping interfaces).
 *
 * Command topics:
 *   ~/left_target_pose   (geometry_msgs/PoseStamped)
 *   ~/right_target_pose  (geometry_msgs/PoseStamped)
 *   ~/joint_homing       (std_msgs/String) -- "left" / "right" / "both"
 *
 * Feedback topics:
 *   ~/joint_position_feedback   (sensor_msgs/JointState)
 *   ~/left_cartesian_state      (geometry_msgs/PoseStamped) -- live EE FK
 *   ~/right_cartesian_state     (geometry_msgs/PoseStamped) -- live EE FK
 *   ~/homing_status             (std_msgs/String)
 *   ~/{left,right}_homing_fk    (geometry_msgs/PoseStamped) -- one-shot
 */
class CartesianPositionControllerPlugin : public controller_interface::ControllerInterface
{
public:
  CartesianPositionControllerPlugin();

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  void leftTargetCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void rightTargetCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void homingCallback(const std_msgs::msg::String::SharedPtr msg);

  // Algorithm library
  std::unique_ptr<ieir_dynamics::IeirDynamics> dynamics_;

  // Parameters
  std::vector<std::string> joint_names_;
  std::vector<double> kp_gains_;        // motor stiffness, one per joint, joints[] order
  std::vector<double> kd_gains_;        // motor damping,   one per joint, joints[] order
  std::string left_ee_frame_;
  std::string right_ee_frame_;
  int ik_max_iter_ = 100;
  double ik_eps_ = 1e-4;
  double ik_damping_ = 1e-3;
  double ik_dt_ = 1.0;
  double position_interpolation_speed_ = 1.0;  // rad/s max joint speed for smooth motion
  // Frame IDs
  int left_frame_id_ = -1;
  int right_frame_id_ = -1;

  // Command interface stride: (position, velocity, stiffness, damping) per joint.
  // State interface stride: (position, velocity) per joint.
  // Layout MUST stay in sync with command_interface_configuration() /
  // state_interface_configuration() below.
  static constexpr size_t kCmdStride = 4;
  static constexpr size_t kStateStride = 2;

  // State storage
  Eigen::VectorXd q_;           // current joint positions (full model)
  Eigen::VectorXd v_;           // current joint velocities (full model)
  Eigen::VectorXd q_desired_;   // desired joint positions from IK
  Eigen::VectorXd q_command_;   // smoothed command being sent
  Eigen::VectorXd q_home_;      // home joint configuration (captured at startup)

  // Cached mapping: joint_names_[i] -> velocity index in full Pinocchio model
  std::vector<int> joint_idx_v_;

  // Realtime buffers for target poses
  realtime_tools::RealtimeBuffer<geometry_msgs::msg::PoseStamped::SharedPtr> rt_left_target_;
  realtime_tools::RealtimeBuffer<geometry_msgs::msg::PoseStamped::SharedPtr> rt_right_target_;
  std::atomic<uint64_t> left_target_seq_{0};
  std::atomic<uint64_t> right_target_seq_{0};
  uint64_t left_target_seq_consumed_{0};
  uint64_t right_target_seq_consumed_{0};
  bool left_target_pending_ = false;
  bool right_target_pending_ = false;

  // Subscribers
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr left_target_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr right_target_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr homing_sub_;

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr feedback_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr homing_status_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr left_homing_fk_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr right_homing_fk_pub_;
  // Continuous live EE pose publishers (FK at q_meas, every tick if a
  // subscriber is present so it costs nothing while no one is listening).
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr left_cartesian_state_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr right_cartesian_state_pub_;

  // Flags
  bool left_target_received_ = false;
  bool right_target_received_ = false;
  bool initial_pose_captured_ = false;
  std::string homing_request_arm_;     // set by callback: "left"/"right"/"both"
  std::atomic<bool> homing_requested_{false};
  bool homing_left_ = false;            // left arm currently homing
  bool homing_right_ = false;           // right arm currently homing

};

}  // namespace ieir_controllers

#endif  // IEIR_CONTROLLERS__CARTESIAN_POSITION_CONTROLLER_PLUGIN_HPP_
