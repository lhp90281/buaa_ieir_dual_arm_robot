#include "ieir_simulation/topic_based_hardware_interface.hpp"
#include <algorithm>
#include <memory>
#include <string>
#include <vector>

namespace ieir_simulation
{

TopicBasedHardwareInterface::TopicBasedHardwareInterface()
{
}

hardware_interface::CallbackReturn TopicBasedHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != 
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Get topic names from URDF parameters (with defaults)
  joint_state_topic_ = "/joint_states";
  command_topic_ = "/ctrl/command";
  gains_topic_ = "/ctrl/gains";

  // Check if custom topic names are provided in URDF
  if (info_.hardware_parameters.find("joint_state_topic") != info_.hardware_parameters.end()) {
    joint_state_topic_ = info_.hardware_parameters["joint_state_topic"];
  }
  if (info_.hardware_parameters.find("command_topic") != info_.hardware_parameters.end()) {
    command_topic_ = info_.hardware_parameters["command_topic"];
  } else if (info_.hardware_parameters.find("effort_command_topic") != info_.hardware_parameters.end()) {
    // Backward compatible with the old effort-only simulator xacro.
    command_topic_ = info_.hardware_parameters["effort_command_topic"];
  }
  if (info_.hardware_parameters.find("gains_topic") != info_.hardware_parameters.end()) {
    gains_topic_ = info_.hardware_parameters["gains_topic"];
  }

  // Extract joint names from URDF
  joint_names_.clear();
  for (const auto & joint : info_.joints) {
    joint_names_.push_back(joint.name);
  }

  // Initialize storage vectors
  joint_positions_.resize(joint_names_.size(), 0.0);
  joint_velocities_.resize(joint_names_.size(), 0.0);
  joint_state_efforts_.resize(joint_names_.size(), 0.0);
  joint_position_commands_.resize(joint_names_.size(), 0.0);
  joint_velocity_commands_.resize(joint_names_.size(), 0.0);
  joint_effort_commands_.resize(joint_names_.size(), 0.0);
  joint_stiffness_.resize(joint_names_.size(), 0.0);
  joint_damping_.resize(joint_names_.size(), 0.0);

  // Create joint name to index map
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    joint_name_to_index_[joint_names_[i]] = i;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn TopicBasedHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Create a ROS 2 node for topic communication
  // Use a unique name to avoid conflicts
  node_ = std::make_shared<rclcpp::Node>("topic_based_hardware_interface");

  // Create subscriber for joint states
  joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    joint_state_topic_, 10,
    std::bind(&TopicBasedHardwareInterface::joint_state_callback, this, std::placeholders::_1));

  // Create publishers for MIT-mode commands.
  command_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(
    command_topic_, 10);
  gains_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(
    gains_topic_, 10);

  RCLCPP_INFO(node_->get_logger(), 
              "TopicBasedHardwareInterface configured with %zu joints", 
              joint_names_.size());
  RCLCPP_INFO(node_->get_logger(), 
              "Subscribing to: %s", joint_state_topic_.c_str());
  RCLCPP_INFO(node_->get_logger(), 
              "Publishing commands to: %s", command_topic_.c_str());
  RCLCPP_INFO(node_->get_logger(),
              "Publishing gains to: %s", gains_topic_.c_str());

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> 
TopicBasedHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < joint_names_.size(); ++i) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        joint_names_[i], 
        hardware_interface::HW_IF_POSITION, 
        &joint_positions_[i]));

    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        joint_names_[i], 
        hardware_interface::HW_IF_VELOCITY, 
        &joint_velocities_[i]));

    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        joint_names_[i],
        hardware_interface::HW_IF_EFFORT,
        &joint_state_efforts_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> 
TopicBasedHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (size_t i = 0; i < joint_names_.size(); ++i) {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        joint_names_[i], 
        hardware_interface::HW_IF_POSITION,
        &joint_position_commands_[i]));

    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        joint_names_[i],
        hardware_interface::HW_IF_VELOCITY,
        &joint_velocity_commands_[i]));

    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        joint_names_[i],
        hardware_interface::HW_IF_EFFORT, 
        &joint_effort_commands_[i]));

    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        joint_names_[i],
        "stiffness",
        &joint_stiffness_[i]));

    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        joint_names_[i],
        "damping",
        &joint_damping_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn TopicBasedHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(node_->get_logger(), "TopicBasedHardwareInterface activated");
  
  // Wait for first joint state message (with timeout)
  RCLCPP_INFO(node_->get_logger(), "Waiting for joint states on %s...", 
              joint_state_topic_.c_str());
  
  auto start_time = node_->now();
  while (!joint_states_received_ && 
         (node_->now() - start_time).seconds() < 5.0) {
    rclcpp::spin_some(node_);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  if (!joint_states_received_) {
    RCLCPP_WARN(node_->get_logger(), 
                "No joint states received after 5 seconds. Proceeding anyway...");
  } else {
    RCLCPP_INFO(node_->get_logger(), "Joint states received successfully");
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn TopicBasedHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Set all active command terms to zero before deactivating.
  std::fill(joint_velocity_commands_.begin(), joint_velocity_commands_.end(), 0.0);
  std::fill(joint_effort_commands_.begin(), joint_effort_commands_.end(), 0.0);
  std::fill(joint_stiffness_.begin(), joint_stiffness_.end(), 0.0);
  std::fill(joint_damping_.begin(), joint_damping_.end(), 0.0);
  
  auto zero_command_msg = std::make_shared<sensor_msgs::msg::JointState>();
  zero_command_msg->header.stamp = node_->now();
  zero_command_msg->name = joint_names_;
  zero_command_msg->position = joint_positions_;
  zero_command_msg->velocity = joint_velocity_commands_;
  zero_command_msg->effort = joint_effort_commands_;
  command_pub_->publish(*zero_command_msg);

  auto zero_gains_msg = std::make_shared<sensor_msgs::msg::JointState>();
  zero_gains_msg->header.stamp = zero_command_msg->header.stamp;
  zero_gains_msg->name = joint_names_;
  zero_gains_msg->position = joint_stiffness_;
  zero_gains_msg->velocity = joint_damping_;
  gains_pub_->publish(*zero_gains_msg);

  RCLCPP_INFO(node_->get_logger(), "TopicBasedHardwareInterface deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

void TopicBasedHardwareInterface::joint_state_callback(
  const sensor_msgs::msg::JointState::SharedPtr msg)
{
  // Update joint positions and velocities from the message
  for (size_t i = 0; i < msg->name.size(); ++i) {
    const std::string& joint_name = msg->name[i];
    
    // Find the index of this joint in our list
    auto it = joint_name_to_index_.find(joint_name);
    if (it != joint_name_to_index_.end()) {
      size_t idx = it->second;
      
      // Update position
      if (i < msg->position.size()) {
        joint_positions_[idx] = msg->position[i];
      }
      
      // Update velocity
      if (i < msg->velocity.size()) {
        joint_velocities_[idx] = msg->velocity[i];
      }

      if (i < msg->effort.size()) {
        joint_state_efforts_[idx] = msg->effort[i];
      }
    }
  }

  last_joint_state_msg_ = msg;
  joint_states_received_ = true;
}

hardware_interface::return_type TopicBasedHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Spin the node to process incoming joint state messages
  rclcpp::spin_some(node_);

  // Joint positions and velocities are updated in the callback
  // No additional processing needed here

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type TopicBasedHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  auto stamp = node_->now();

  auto command_msg = std::make_shared<sensor_msgs::msg::JointState>();
  command_msg->header.stamp = stamp;
  command_msg->name = joint_names_;
  command_msg->position = joint_position_commands_;
  command_msg->velocity = joint_velocity_commands_;
  command_msg->effort = joint_effort_commands_;

  auto gains_msg = std::make_shared<sensor_msgs::msg::JointState>();
  gains_msg->header.stamp = stamp;
  gains_msg->name = joint_names_;
  gains_msg->position = joint_stiffness_;
  gains_msg->velocity = joint_damping_;

  command_pub_->publish(*command_msg);
  gains_pub_->publish(*gains_msg);

  return hardware_interface::return_type::OK;
}

}  // namespace ieir_simulation

// Export the plugin
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  ieir_simulation::TopicBasedHardwareInterface,
  hardware_interface::SystemInterface)
