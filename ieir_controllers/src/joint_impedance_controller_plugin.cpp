#include "ieir_controllers/joint_impedance_controller_plugin.hpp"
#include <algorithm>
#include <cmath>
#include <memory>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

#include "controller_interface/helpers.hpp"

namespace ieir_controllers
{

JointImpedanceControllerPlugin::JointImpedanceControllerPlugin()
: controller_interface::ControllerInterface()
{
}

controller_interface::InterfaceConfiguration 
JointImpedanceControllerPlugin::command_interface_configuration() const
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
JointImpedanceControllerPlugin::state_interface_configuration() const
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

controller_interface::CallbackReturn JointImpedanceControllerPlugin::on_init()
{
  try {
    // Declare parameters
    auto_declare<std::vector<std::string>>("joints", std::vector<std::string>());
    auto_declare<double>("default_stiffness", 20.0);
    auto_declare<double>("default_damping", 5.0);
    auto_declare<double>("max_effort", 50.0);
    auto_declare<double>("position_error_limit", 0.087);
    auto_declare<double>("velocity_filter_alpha", 0.99);
    auto_declare<double>("velocity_saturation", 5.0);
    
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception during on_init: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointImpedanceControllerPlugin::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Get parameters
  joint_names_ = get_node()->get_parameter("joints").as_string_array();
  default_stiffness_ = get_node()->get_parameter("default_stiffness").as_double();
  default_damping_ = get_node()->get_parameter("default_damping").as_double();
  max_effort_ = get_node()->get_parameter("max_effort").as_double();
  position_error_limit_ = get_node()->get_parameter("position_error_limit").as_double();
  velocity_filter_alpha_ = get_node()->get_parameter("velocity_filter_alpha").as_double();
  velocity_saturation_ = get_node()->get_parameter("velocity_saturation").as_double();

  if (joint_names_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "No joints specified in 'joints' parameter!");
    return controller_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(get_node()->get_logger(), 
              "Configuring JointImpedanceController with %zu joints", joint_names_.size());

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
  // Don't use neutral configuration - wait for first update() to capture current pose
  q_desired_ = Eigen::VectorXd::Zero(dynamics_->getNq());
  v_desired_ = Eigen::VectorXd::Zero(dynamics_->getNv());

  // Build cached joint name -> Pinocchio velocity index mapping
  joint_idx_v_.resize(joint_names_.size(), -1);
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    for (int j = 1; j < dynamics_->getNumJoints() + 1; ++j) {
      if (dynamics_->getJointName(j) == joint_names_[i]) {
        joint_idx_v_[i] = dynamics_->getJointIdxV(j);
        break;
      }
    }
    if (joint_idx_v_[i] < 0) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Joint '%s' not found in Pinocchio model!", joint_names_[i].c_str());
    }
  }

  // Initialize gain vectors and per-joint gains map
  stiffness_vec_ = Eigen::VectorXd::Constant(dynamics_->getNv(), default_stiffness_);
  damping_vec_ = Eigen::VectorXd::Constant(dynamics_->getNv(), default_damping_);

  // Load per-joint gains from parameters
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    const std::string& joint_name = joint_names_[i];
    
    // Declare per-joint gain parameters if they exist
    std::string stiffness_param = "gains." + joint_name + ".stiffness";
    std::string damping_param = "gains." + joint_name + ".damping";
    
    double joint_stiffness = default_stiffness_;
    double joint_damping = default_damping_;
    
    // Try to get per-joint parameters
    if (get_node()->has_parameter(stiffness_param)) {
      joint_stiffness = get_node()->get_parameter(stiffness_param).as_double();
    } else {
      get_node()->declare_parameter(stiffness_param, default_stiffness_);
    }
    
    if (get_node()->has_parameter(damping_param)) {
      joint_damping = get_node()->get_parameter(damping_param).as_double();
    } else {
      get_node()->declare_parameter(damping_param, default_damping_);
    }
    
    // Get effort limit from dynamics model (from URDF)
    double effort_limit = max_effort_;
    int idx_v = joint_idx_v_[i];
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
    joint_gains_[joint_name] = {joint_stiffness, joint_damping, effort_limit};
    
    // Update gain vectors using Pinocchio velocity index (NOT loop index i)
    if (idx_v >= 0 && idx_v < stiffness_vec_.size()) {
      stiffness_vec_[idx_v] = joint_stiffness;
      damping_vec_[idx_v] = joint_damping;
    }
    
    RCLCPP_INFO(get_node()->get_logger(), 
                "Joint %s: Kp=%.2f, Kd=%.2f, Effort Limit=%.2f, idx_v=%d",
                joint_name.c_str(), joint_stiffness, joint_damping, effort_limit, idx_v);
  }

  // Create subscriber for target commands with realtime buffer
  target_sub_ = get_node()->create_subscription<sensor_msgs::msg::JointState>(
    "~/target_joint_states", 10,
    [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
      this->targetCommandCallback(msg);
    });

  // Create feedback publisher (optional, for debugging)
  feedback_pub_ = get_node()->create_publisher<sensor_msgs::msg::JointState>(
    "~/effort_feedback", 10);

  // Initialize realtime buffer with empty message
  auto empty_msg = std::make_shared<sensor_msgs::msg::JointState>();
  rt_target_buffer_.writeFromNonRT(empty_msg);

  RCLCPP_INFO(get_node()->get_logger(), "JointImpedanceController configured successfully");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointImpedanceControllerPlugin::on_activate(
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

  // Reset flags
  target_received_ = false;
  initial_pose_captured_ = false;
  velocity_initialized_ = false;
  warmup_counter_ = 0;

  RCLCPP_INFO(get_node()->get_logger(), 
              "JointImpedanceController activated. Waiting to capture initial pose...");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointImpedanceControllerPlugin::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Set all command interfaces to zero before releasing
  for (auto & command_interface : command_interfaces_) {
    command_interface.set_value(0.0);
  }

  RCLCPP_INFO(get_node()->get_logger(), "JointImpedanceController deactivated");
  return controller_interface::CallbackReturn::SUCCESS;
}

void JointImpedanceControllerPlugin::targetCommandCallback(
  const sensor_msgs::msg::JointState::SharedPtr msg)
{
  // This callback runs in non-realtime context
  // Write to realtime buffer for safe access in update()
  rt_target_buffer_.writeFromNonRT(msg);
  target_received_ = true;
}

controller_interface::return_type JointImpedanceControllerPlugin::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // Read joint states from state_interfaces and map to full state vector
  // Note: q_ and v_ are size nq/nv (18), but we only control 14 joints
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    double q_current = state_interfaces_[2 * i].get_value();      // position
    double v_current = state_interfaces_[2 * i + 1].get_value();  // velocity
    
    int idx_v = joint_idx_v_[i];
    
    if (idx_v >= 0 && idx_v < q_.size()) {
      q_[idx_v] = q_current;
      v_[idx_v] = v_current;
      
      // Apply low-pass filter to velocity (filtering for smooth damping)
      v_filtered_[idx_v] = (1.0 - velocity_filter_alpha_) * v_current + 
                           velocity_filter_alpha_ * v_filtered_[idx_v];
    }
  }

  // Initialize v_filtered_ from first real measurement to avoid startup transient
  if (!velocity_initialized_) {
    v_filtered_ = v_;
    velocity_initialized_ = true;
  }

  // Update dynamics state with FILTERED velocity, then compute gravity
  dynamics_->updateState(q_, v_filtered_);
  Eigen::VectorXd tau_gravity = dynamics_->computeGravity();

  // Capture initial pose if not done yet
  if (!initial_pose_captured_) {
    q_desired_ = q_;
    v_desired_ = Eigen::VectorXd::Zero(dynamics_->getNv());
    initial_pose_captured_ = true;
    RCLCPP_INFO(get_node()->get_logger(), "Initial pose captured for impedance control");
    
    // Log gravity torques for diagnostics
    std::ostringstream oss;
    oss << "Gravity torques at initial pose: [";
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      int idx_v = joint_idx_v_[i];
      if (idx_v >= 0 && idx_v < tau_gravity.size()) {
        oss << joint_names_[i] << "=" << std::fixed << std::setprecision(2) << tau_gravity[idx_v];
      }
      if (i + 1 < joint_names_.size()) oss << ", ";
    }
    oss << "]";
    RCLCPP_INFO(get_node()->get_logger(), "%s", oss.str().c_str());
  }

  // Gravity-only warmup: apply only gravity compensation for first WARMUP_CYCLES
  // This lets the system settle before engaging spring/damping terms
  if (warmup_counter_ < WARMUP_CYCLES) {
    warmup_counter_++;
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      int idx_v = joint_idx_v_[i];
      double effort_limit = joint_gains_[joint_names_[i]].effort_limit;
      if (idx_v >= 0 && idx_v < tau_gravity.size()) {
        double clamped = std::max(-effort_limit, std::min(tau_gravity[idx_v], effort_limit));
        command_interfaces_[i].set_value(clamped);
      } else {
        command_interfaces_[i].set_value(0.0);
      }
    }
    if (warmup_counter_ == WARMUP_CYCLES) {
      // Re-capture pose after warmup (arm may have settled slightly)
      q_desired_ = q_;
      RCLCPP_INFO(get_node()->get_logger(),
                  "Warmup complete (%d cycles). Re-captured pose. Engaging impedance control.",
                  WARMUP_CYCLES);
    }
    return controller_interface::return_type::OK;
  }

  // Read target command from realtime buffer
  auto target_msg = rt_target_buffer_.readFromRT();
  if (target_msg && (*target_msg) && target_received_) {
    // Update desired state from target message
    // Only update joints specified in the message, others maintain previous target
    for (size_t i = 0; i < (*target_msg)->name.size(); ++i) {
      const std::string& name = (*target_msg)->name[i];
      
      // Use cached mapping: find idx_v for this target joint name
      int idx_v = -1;
      for (size_t k = 0; k < joint_names_.size(); ++k) {
        if (joint_names_[k] == name) {
          idx_v = joint_idx_v_[k];
          break;
        }
      }
      
      if (idx_v >= 0 && idx_v < q_desired_.size()) {
        if (i < (*target_msg)->position.size()) {
          q_desired_[idx_v] = (*target_msg)->position[i];
        }
        
        if (i < (*target_msg)->velocity.size()) {
          v_desired_[idx_v] = (*target_msg)->velocity[i];
        }
      }
    }
  }

  // Compute position and velocity errors
  Eigen::VectorXd q_error = q_desired_ - q_;
  
  // Clamp position error to prevent excessive torque
  for (int i = 0; i < q_error.size(); ++i) {
    q_error[i] = std::max(-position_error_limit_, 
                          std::min(q_error[i], position_error_limit_));
  }
  
  // Velocity error using FILTERED velocity (v_filtered_ was set via updateState)
  Eigen::VectorXd v_error = v_desired_ - v_filtered_;
  
  // Clamp velocity error to prevent noise amplification in damping term
  for (int i = 0; i < v_error.size(); ++i) {
    v_error[i] = std::max(-velocity_saturation_, 
                          std::min(v_error[i], velocity_saturation_));
  }
  
  // Manually compute impedance torque: tau = Kp*q_error + Kd*v_error
  // This ensures clamped position error and filtered velocity are used
  Eigen::VectorXd tau_position = stiffness_vec_.cwiseProduct(q_error);
  Eigen::VectorXd tau_damping = damping_vec_.cwiseProduct(v_error);
  Eigen::VectorXd tau_impedance = tau_position + tau_damping;

  // Total torque (used for feedback only; actual command uses damping-priority below)
  Eigen::VectorXd tau_total = tau_gravity + tau_impedance;

  // Damping-priority torque allocation:
  // Clamp (gravity + spring) FIRST to reserve headroom for damping.
  // This prevents torque saturation from eating the damping budget,
  // which would cause undamped limit-cycle oscillations.
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    int idx_v = joint_idx_v_[i];
    
    if (idx_v >= 0 && idx_v < tau_total.size()) {
      double effort_limit = joint_gains_[joint_names_[i]].effort_limit;
      double kd = damping_vec_[idx_v];
      
      // Reserve headroom for damping: Kd * velocity_saturation
      double damping_reserve = kd * velocity_saturation_;
      double base_limit = std::max(0.0, effort_limit - damping_reserve);
      
      // Clamp gravity + spring to the reduced limit
      double tau_base = tau_gravity[idx_v] + tau_position[idx_v];
      tau_base = std::max(-base_limit, std::min(tau_base, base_limit));
      
      // Add damping (full range available within remaining headroom)
      double tau_cmd = tau_base + tau_damping[idx_v];
      
      // Final safety clamp to effort limit
      tau_cmd = std::max(-effort_limit, std::min(tau_cmd, effort_limit));
      
      command_interfaces_[i].set_value(tau_cmd);
    } else {
      command_interfaces_[i].set_value(0.0);
    }
  }

  // Publish feedback (optional, for debugging)
  if (feedback_pub_ && feedback_pub_->get_subscription_count() > 0) {
    auto feedback_msg = std::make_shared<sensor_msgs::msg::JointState>();
    feedback_msg->header.stamp = get_node()->now();
    feedback_msg->name = joint_names_;
    feedback_msg->effort.resize(joint_names_.size());
    
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      int idx_v = joint_idx_v_[i];
      
      if (idx_v >= 0 && idx_v < tau_impedance.size()) {
        feedback_msg->effort[i] = tau_impedance[idx_v];
      } else {
        feedback_msg->effort[i] = 0.0;
      }
    }
    
    feedback_pub_->publish(*feedback_msg);
  }

  return controller_interface::return_type::OK;
}

}  // namespace ieir_controllers

// Export the plugin
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  ieir_controllers::JointImpedanceControllerPlugin,
  controller_interface::ControllerInterface)
