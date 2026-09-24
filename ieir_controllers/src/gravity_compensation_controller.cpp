#include "ieir_controllers/gravity_compensation_controller.hpp"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/helpers.hpp"
#include "yaml-cpp/yaml.h"

namespace ieir_controllers
{

GravityCompensationController::GravityCompensationController()
: controller_interface::ControllerInterface()
{
}

controller_interface::InterfaceConfiguration 
GravityCompensationController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // Request effort command interfaces for all joints
  for (const auto & joint_name : joint_names_) {
    config.names.push_back(joint_name + "/effort");
  }

  return config;
}

controller_interface::InterfaceConfiguration 
GravityCompensationController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // Request position and velocity state interfaces for all joints
  for (const auto & joint_name : joint_names_) {
    config.names.push_back(joint_name + "/position");
    config.names.push_back(joint_name + "/velocity");
  }

  return config;
}

controller_interface::CallbackReturn GravityCompensationController::on_init()
{
  try {
    // Declare parameters
    auto_declare<std::vector<std::string>>("joints", std::vector<std::string>());
    auto_declare<double>("max_effort", 50.0);
    auto_declare<double>("velocity_filter_alpha", 0.99);

    // Per-joint gravity scale. Default empty -> all 1.0.
    // If specified, length must equal joints.size().
    auto_declare<std::vector<double>>("gravity_gains", std::vector<double>());

    // Friction compensation (optional, off by default)
    auto_declare<bool>("friction_compensation_enabled", false);
    auto_declare<std::string>("friction_model_yaml", std::string());
    auto_declare<std::vector<std::string>>("motor_types", std::vector<std::string>());
    // Scalar friction_gain stays as a scalar fallback for the per-joint
    // friction_gains array (used when the array is unset / empty).
    auto_declare<double>("friction_gain", 0.7);
    auto_declare<std::vector<double>>("friction_gains", std::vector<double>());
    auto_declare<double>("friction_deadband", 0.05);

  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception during on_init: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

bool GravityCompensationController::load_friction_model(
  const std::string & yaml_path,
  const std::vector<std::string> & motor_types)
{
  joint_frictions_.assign(joint_names_.size(), JointFriction{});

  if (motor_types.size() != joint_names_.size()) {
    RCLCPP_ERROR(get_node()->get_logger(),
      "friction_compensation_enabled but motor_types size (%zu) != joints size (%zu)",
      motor_types.size(), joint_names_.size());
    return false;
  }
  if (yaml_path.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(),
      "friction_compensation_enabled but friction_model_yaml is empty");
    return false;
  }

  std::ifstream f(yaml_path);
  if (!f.good()) {
    RCLCPP_ERROR(get_node()->get_logger(),
      "Cannot open friction_model_yaml '%s'", yaml_path.c_str());
    return false;
  }

  YAML::Node root;
  try {
    root = YAML::LoadFile(yaml_path);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(),
      "Failed to parse friction_model_yaml '%s': %s", yaml_path.c_str(), e.what());
    return false;
  }

  size_t hit = 0;
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    const std::string & mt = motor_types[i];
    if (!root[mt]) {
      RCLCPP_ERROR(get_node()->get_logger(),
        "motor_type '%s' (joint '%s') not found in %s",
        mt.c_str(), joint_names_[i].c_str(), yaml_path.c_str());
      return false;
    }
    const auto & m = root[mt];
    JointFriction jf;
    try {
      jf.viscous     = m["viscous"].as<double>();
      jf.coulomb_pos = m["coulomb_pos"].as<double>();
      jf.coulomb_neg = m["coulomb_neg"].as<double>();
      jf.valid = std::isfinite(jf.viscous) &&
                 std::isfinite(jf.coulomb_pos) &&
                 std::isfinite(jf.coulomb_neg);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_node()->get_logger(),
        "motor_type '%s' missing/invalid friction fields: %s", mt.c_str(), e.what());
      return false;
    }
    if (!jf.valid) {
      RCLCPP_ERROR(get_node()->get_logger(),
        "motor_type '%s' has non-finite friction parameters", mt.c_str());
      return false;
    }
    joint_frictions_[i] = jf;
    RCLCPP_INFO(get_node()->get_logger(),
      "Friction[%s -> %s]: gain=%.3f viscous=%.4f coulomb_pos=%.3f coulomb_neg=%.3f",
      joint_names_[i].c_str(), mt.c_str(), friction_gains_[i],
      jf.viscous, jf.coulomb_pos, jf.coulomb_neg);
    ++hit;
  }
  RCLCPP_INFO(get_node()->get_logger(),
    "Friction compensation loaded for %zu/%zu joints (per-joint gain, deadband=%.3f)",
    hit, joint_names_.size(), friction_deadband_);
  return true;
}

controller_interface::CallbackReturn GravityCompensationController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Get parameters
  joint_names_ = get_node()->get_parameter("joints").as_string_array();
  max_effort_ = get_node()->get_parameter("max_effort").as_double();
  velocity_filter_alpha_ = get_node()->get_parameter("velocity_filter_alpha").as_double();

  if (joint_names_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "No joints specified in 'joints' parameter!");
    return controller_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(get_node()->get_logger(), 
              "Configuring GravityCompensationController with %zu joints", joint_names_.size());

  // Get URDF from robot_description parameter
  std::string robot_description;
  if (!get_node()->get_parameter("robot_description", robot_description)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to get 'robot_description' parameter!");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (robot_description.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "robot_description parameter is empty!");
    return controller_interface::CallbackReturn::ERROR;
  }

  // Initialize dynamics library
  dynamics_ = std::make_unique<ieir_dynamics::IeirDynamics>();
  if (!dynamics_->initFromURDF(robot_description)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to initialize dynamics from URDF!");
    return controller_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(get_node()->get_logger(), 
              "Dynamics initialized. nq=%d, nv=%d", 
              dynamics_->getNq(), dynamics_->getNv());

  // Initialize state vectors
  q_ = Eigen::VectorXd::Zero(dynamics_->getNq());
  v_ = Eigen::VectorXd::Zero(dynamics_->getNv());
  v_filtered_ = Eigen::VectorXd::Zero(dynamics_->getNv());

  // Build joint name to velocity index mapping
  std::map<std::string, int> joint_name_to_idx_v;
  for (int j = 1; j < dynamics_->getNumJoints() + 1; ++j) {
    std::string name = dynamics_->getJointName(j);
    int idx_v = dynamics_->getJointIdxV(j);
    if (!name.empty() && idx_v >= 0) {
      joint_name_to_idx_v[name] = idx_v;
    }
  }

  // ---- Per-joint gravity gains (multiplicative scale on tau_gravity[i]) ----
  // If user provided a 'gravity_gains' array it must match joints.size().
  // Empty/missing -> default to all 1.0 (no scaling, original behavior).
  const auto gains_param = get_node()->get_parameter("gravity_gains").as_double_array();
  gravity_gains_.assign(joint_names_.size(), 1.0);
  if (!gains_param.empty()) {
    if (gains_param.size() != joint_names_.size()) {
      RCLCPP_ERROR(get_node()->get_logger(),
        "gravity_gains size (%zu) != joints size (%zu)",
        gains_param.size(), joint_names_.size());
      return controller_interface::CallbackReturn::ERROR;
    }
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      if (!std::isfinite(gains_param[i])) {
        RCLCPP_ERROR(get_node()->get_logger(),
          "gravity_gains[%zu] for joint '%s' is non-finite", i,
          joint_names_[i].c_str());
        return controller_interface::CallbackReturn::ERROR;
      }
      gravity_gains_[i] = gains_param[i];
    }
  }

  // Load per-joint effort limits and verify all joints exist in model. Also
  // cache idx_v per joint so update() does not have to scan the Pinocchio
  // model on every realtime tick.
  joint_idx_v_.assign(joint_names_.size(), -1);
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    const std::string& joint_name = joint_names_[i];

    // Check if joint exists in model
    auto it = joint_name_to_idx_v.find(joint_name);
    if (it == joint_name_to_idx_v.end()) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Joint %s not found in dynamics model!", joint_name.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    joint_idx_v_[i] = it->second;

    // Get effort limit from dynamics model (from URDF)
    double effort_limit = max_effort_;
    for (int j = 1; j < dynamics_->getNumJoints() + 1; ++j) {
      if (dynamics_->getJointName(j) == joint_name) {
        double urdf_limit = dynamics_->getEffortLimit(j);
        if (urdf_limit > 0) {
          effort_limit = urdf_limit;
        }
        break;
      }
    }

    // Store in map
    joint_gains_[joint_name] = {effort_limit};

    RCLCPP_INFO(get_node()->get_logger(),
                "Joint %s (idx_v=%d): Effort Limit=%.2f, gravity_gain=%.3f "
                "(pure gravity compensation, Kp=0, Kd=0)",
                joint_name.c_str(), joint_idx_v_[i], effort_limit,
                gravity_gains_[i]);
  }

  // ---- friction compensation (optional) ----
  friction_enabled_ = get_node()->get_parameter("friction_compensation_enabled").as_bool();
  friction_deadband_ = get_node()->get_parameter("friction_deadband").as_double();

  // Per-joint friction_gains. If the array is non-empty its length must
  // equal joints.size(); otherwise we broadcast the scalar 'friction_gain'
  // (the legacy single-knob behavior) to every joint.
  const double scalar_friction_gain =
    get_node()->get_parameter("friction_gain").as_double();
  const auto fgains_param =
    get_node()->get_parameter("friction_gains").as_double_array();
  friction_gains_.assign(joint_names_.size(), scalar_friction_gain);
  if (!fgains_param.empty()) {
    if (fgains_param.size() != joint_names_.size()) {
      RCLCPP_ERROR(get_node()->get_logger(),
        "friction_gains size (%zu) != joints size (%zu)",
        fgains_param.size(), joint_names_.size());
      return controller_interface::CallbackReturn::ERROR;
    }
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      if (!std::isfinite(fgains_param[i])) {
        RCLCPP_ERROR(get_node()->get_logger(),
          "friction_gains[%zu] for joint '%s' is non-finite", i,
          joint_names_[i].c_str());
        return controller_interface::CallbackReturn::ERROR;
      }
      friction_gains_[i] = fgains_param[i];
    }
  }

  if (friction_enabled_) {
    const std::string yaml_path =
      get_node()->get_parameter("friction_model_yaml").as_string();
    const std::vector<std::string> motor_types =
      get_node()->get_parameter("motor_types").as_string_array();
    if (!load_friction_model(yaml_path, motor_types)) {
      RCLCPP_WARN(get_node()->get_logger(),
        "Friction compensation requested but failed to load; disabling.");
      friction_enabled_ = false;
      joint_frictions_.assign(joint_names_.size(), JointFriction{});
    }
  } else {
    joint_frictions_.assign(joint_names_.size(), JointFriction{});
    RCLCPP_INFO(get_node()->get_logger(), "Friction compensation disabled.");
  }

  RCLCPP_INFO(get_node()->get_logger(), "GravityCompensationController configured successfully");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn GravityCompensationController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Verify that all required interfaces are available
  if (state_interfaces_.size() != joint_names_.size() * 2) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Expected %zu state interfaces, got %zu",
                 joint_names_.size() * 2, state_interfaces_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  if (command_interfaces_.size() != joint_names_.size()) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Expected %zu command interfaces, got %zu",
                 joint_names_.size(), command_interfaces_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(get_node()->get_logger(), 
              "GravityCompensationController activated. Robot should feel weightless.");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn GravityCompensationController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Set all command interfaces to zero before releasing
  for (auto & command_interface : command_interfaces_) {
    command_interface.set_value(0.0);
  }

  RCLCPP_INFO(get_node()->get_logger(), "GravityCompensationController deactivated");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type GravityCompensationController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // Read joint states from state_interfaces and map to full state vector.
  // Note: q_ and v_ are size nq/nv (18), but we only control 14 joints.
  // idx_v for each controlled joint was cached in on_configure().
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    const double q_current = state_interfaces_[2 * i].get_value();      // position
    const double v_current = state_interfaces_[2 * i + 1].get_value();  // velocity
    const int idx_v = joint_idx_v_[i];

    if (idx_v >= 0 && idx_v < q_.size()) {
      q_[idx_v] = q_current;
      v_[idx_v] = v_current;

      // Apply low-pass filter to velocity (same as impedance controller for consistency)
      v_filtered_[idx_v] = (1.0 - velocity_filter_alpha_) * v_current +
                           velocity_filter_alpha_ * v_filtered_[idx_v];
    }
  }

  // Update dynamics state with FILTERED velocity (consistent with impedance controller)
  dynamics_->updateState(q_, v_filtered_);

  // Compute gravity compensation
  Eigen::VectorXd tau_gravity = dynamics_->computeGravity();

  // Apply per-joint effort limits and write to command interfaces.
  // tau_total = gravity_gain[i] * tau_gravity[idx_v] + tau_friction
  // (gravity_gain[i] is the user-tunable multiplicative scale; default 1.0).
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    const int idx_v = joint_idx_v_[i];

    if (idx_v >= 0 && idx_v < tau_gravity.size()) {
      double tau_total = gravity_gains_[i] * tau_gravity[idx_v];

      // Friction compensation: friction_gains[i] * [sign(v)*coulomb_dir + viscous*v]
      // applied only outside the velocity deadband to avoid jitter creep.
      if (friction_enabled_ && joint_frictions_[i].valid) {
        const double v = v_filtered_[idx_v];
        if (std::abs(v) > friction_deadband_) {
          const auto & fm = joint_frictions_[i];
          const double tau_coulomb = (v > 0.0) ? fm.coulomb_pos : -fm.coulomb_neg;
          const double tau_friction =
            friction_gains_[i] * (tau_coulomb + fm.viscous * v);
          tau_total += tau_friction;
        }
      }

      const double effort_limit = joint_gains_[joint_names_[i]].effort_limit;
      const double clamped_effort =
        std::max(-effort_limit, std::min(tau_total, effort_limit));
      command_interfaces_[i].set_value(clamped_effort);
    } else {
      command_interfaces_[i].set_value(0.0);
    }
  }

  return controller_interface::return_type::OK;
}

}  // namespace ieir_controllers

// Export the plugin
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  ieir_controllers::GravityCompensationController,
  controller_interface::ControllerInterface)
