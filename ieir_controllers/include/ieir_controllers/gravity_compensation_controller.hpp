#ifndef IEIR_CONTROLLERS__GRAVITY_COMPENSATION_CONTROLLER_HPP_
#define IEIR_CONTROLLERS__GRAVITY_COMPENSATION_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <map>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "ieir_controllers/ieir_dynamics.hpp"

namespace ieir_controllers
{

/**
 * @brief Gravity Compensation Controller for ros2_control
 * 
 * This controller implements pure gravity compensation (zero stiffness, zero damping).
 * Formula: tau = g(q)
 * 
 * This is essentially the impedance controller with Kp=0 and Kd=0.
 * The robot will feel "weightless" and can be freely moved by hand.
 */
class GravityCompensationController : public controller_interface::ControllerInterface
{
public:
  GravityCompensationController();

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
    double effort_limit;
  };

  // Per-joint friction model (parameters of one motor type, copied per joint
  // so we do not pay a map lookup inside the realtime update loop).
  struct JointFriction {
    bool valid{false};
    double viscous{0.0};
    double coulomb_pos{0.0};
    double coulomb_neg{0.0};
  };

  /// Load friction_model.yaml and populate joint_frictions_ aligned with
  /// joint_names_ using motor_types. Returns false on any unrecoverable
  /// error (missing file, missing key for an enabled joint).
  bool load_friction_model(const std::string & yaml_path,
                           const std::vector<std::string> & motor_types);

  // Algorithm library
  std::unique_ptr<ieir_dynamics::IeirDynamics> dynamics_;

  // Parameters
  std::vector<std::string> joint_names_;
  std::map<std::string, JointGains> joint_gains_;
  double max_effort_ = 50.0;
  double velocity_filter_alpha_ = 0.99;

  // Per-joint multiplicative scale on the gravity term. Stored in the same
  // order as joint_names_ for O(1) indexing in update(). Default = 1.0
  // (no change). >1.0 to compensate more (e.g. URDF mass under-estimated),
  // <1.0 to compensate less. Loaded from the 'gravity_gains' parameter.
  std::vector<double> gravity_gains_;

  // idx_v cache aligned with joint_names_ to avoid scanning the Pinocchio
  // model on every update() iteration.
  std::vector<int> joint_idx_v_;

  // Friction compensation (optional, off by default)
  bool friction_enabled_ = false;
  // Per-joint friction compensation gain. Loaded from the 'friction_gains'
  // array parameter; falls back to the scalar 'friction_gain' (broadcast to
  // every joint) when the array is unset / empty. Same indexing as
  // joint_names_.
  std::vector<double> friction_gains_;
  double friction_deadband_ = 0.05;
  std::vector<JointFriction> joint_frictions_;  // size == joint_names_.size()

  // State storage
  Eigen::VectorXd q_;
  Eigen::VectorXd v_;
  Eigen::VectorXd v_filtered_;
};

}  // namespace ieir_controllers

#endif  // IEIR_CONTROLLERS__GRAVITY_COMPENSATION_CONTROLLER_HPP_
