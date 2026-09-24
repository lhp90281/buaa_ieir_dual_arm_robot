#ifndef IEIR_SIMULATION__TOPIC_BASED_HARDWARE_INTERFACE_HPP_
#define IEIR_SIMULATION__TOPIC_BASED_HARDWARE_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <map>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace ieir_simulation
{

/**
 * @brief Topic-based Hardware Interface for ros2_control
 * 
 * This is a bridge that allows ros2_control to work with systems that
 * communicate via topics (like simulators or real robots with topic-based drivers).
 * 
 * It subscribes to /joint_states and publishes MIT-style commands to
 * /ctrl/command plus /ctrl/gains, making it compatible with the MuJoCo bridge.
 * 
 * This approach is useful for:
 * - Simulators that use topic communication (MuJoCo, Gazebo Classic, etc.)
 * - Real robots with topic-based low-level drivers
 * - Gradual migration from topic-based to ros2_control architecture
 */
class TopicBasedHardwareInterface : public hardware_interface::SystemInterface
{
public:
  TopicBasedHardwareInterface();
  virtual ~TopicBasedHardwareInterface() = default;

  /**
   * @brief Initialize the hardware interface
   */
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  /**
   * @brief Configure the hardware interface
   */
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Export state interfaces (position, velocity)
   */
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  /**
   * @brief Export command interfaces (position, velocity, effort, stiffness, damping)
   */
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  /**
   * @brief Activate the hardware interface
   */
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Deactivate the hardware interface
   */
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Read joint states from /joint_states topic
   */
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  /**
   * @brief Write MIT-style commands to simulator command topics
   */
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  /**
   * @brief Callback for /joint_states topic
   */
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);

  // ROS 2 node for communication
  rclcpp::Node::SharedPtr node_;

  // Topic subscriber and publisher
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr command_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr gains_pub_;

  // Joint names (from URDF)
  std::vector<std::string> joint_names_;

  // State storage (exposed to controllers via state_interfaces)
  std::vector<double> joint_positions_;
  std::vector<double> joint_velocities_;
  std::vector<double> joint_state_efforts_;

  // Command storage (written by controllers via command_interfaces)
  std::vector<double> joint_position_commands_;
  std::vector<double> joint_velocity_commands_;
  std::vector<double> joint_effort_commands_;
  std::vector<double> joint_stiffness_;
  std::vector<double> joint_damping_;

  // Map joint names to indices for fast lookup
  std::map<std::string, size_t> joint_name_to_index_;

  // Latest joint state message
  sensor_msgs::msg::JointState::SharedPtr last_joint_state_msg_;

  // Topic names (configurable via URDF)
  std::string joint_state_topic_;
  std::string command_topic_;
  std::string gains_topic_;

  // Flag to check if we've received joint states
  bool joint_states_received_ = false;
};

}  // namespace ieir_simulation

#endif  // IEIR_SIMULATION__TOPIC_BASED_HARDWARE_INTERFACE_HPP_
