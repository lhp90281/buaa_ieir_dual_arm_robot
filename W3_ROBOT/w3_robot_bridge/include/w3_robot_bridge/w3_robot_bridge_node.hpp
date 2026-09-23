#pragma once

#include "w3_robot_bridge/motor_bridge.hpp"
#include "w3_robot_bridge/tigertoe_protocol.hpp"

#include <rclcpp/rclcpp.hpp>

#include "w3_robot_bridge/msg/motor_command.hpp"
#include "w3_robot_bridge/msg/motor_command_array.hpp"
#include "w3_robot_bridge/msg/motor_state.hpp"
#include "w3_robot_bridge/msg/motor_state_array.hpp"
#include "w3_robot_bridge/srv/enable_motor.hpp"
#include "w3_robot_bridge/srv/disable_motor.hpp"
#include "w3_robot_bridge/srv/set_zero.hpp"

#include <chrono>
#include <memory>
#include <string>

namespace w3_robot_bridge {

/// ROS 2 node for the W3 Robot Bridge.
///
/// Interfaces:
///   Subscribers:
///     ~/command  (MotorCommand)       — single motor control
///     ~/commands (MotorCommandArray)  — batch group control
///   Publisher:
///     ~/state    (MotorStateArray)    — all motor states @ 200Hz
///   Services:
///     ~/enable   (EnableMotor)        — enable motor(s)
///     ~/disable  (DisableMotor)       — disable motor(s)
///     ~/zero     (SetZero)            — set current position as zero
///
/// channel=255 → all channels, motor_index=255 → all motors on channel.
class W3RobotBridgeNode : public rclcpp::Node {
public:
    W3RobotBridgeNode();
    ~W3RobotBridgeNode() override;

private:
    // ── Initialization ─────────────────────────────────────────
    void declare_parameters();
    bool load_config();
    void setup_ros_interfaces();

    // ── Callbacks ──────────────────────────────────────────────
    void on_single_command(const w3_robot_bridge::msg::MotorCommand::SharedPtr msg);
    void on_batch_command(const w3_robot_bridge::msg::MotorCommandArray::SharedPtr msg);

    void on_enable(
        const w3_robot_bridge::srv::EnableMotor::Request::SharedPtr req,
        w3_robot_bridge::srv::EnableMotor::Response::SharedPtr res);
    void on_disable(
        const w3_robot_bridge::srv::DisableMotor::Request::SharedPtr req,
        w3_robot_bridge::srv::DisableMotor::Response::SharedPtr res);
    void on_set_zero(
        const w3_robot_bridge::srv::SetZero::Request::SharedPtr req,
        w3_robot_bridge::srv::SetZero::Response::SharedPtr res);

    // ── Timers ─────────────────────────────────────────────────
    void publish_states();
    void watchdog_tick();

    // ── Members ────────────────────────────────────────────────
    MotorBridge bridge_{};
    std::string config_path_;

    // ROS interfaces
    rclcpp::Subscription<w3_robot_bridge::msg::MotorCommand>::SharedPtr      cmd_sub_;
    rclcpp::Subscription<w3_robot_bridge::msg::MotorCommandArray>::SharedPtr cmds_sub_;
    rclcpp::Publisher<w3_robot_bridge::msg::MotorStateArray>::SharedPtr      state_pub_;
    rclcpp::Service<w3_robot_bridge::srv::EnableMotor>::SharedPtr            enable_srv_;
    rclcpp::Service<w3_robot_bridge::srv::DisableMotor>::SharedPtr           disable_srv_;
    rclcpp::Service<w3_robot_bridge::srv::SetZero>::SharedPtr                zero_srv_;
    rclcpp::TimerBase::SharedPtr state_timer_;
    rclcpp::TimerBase::SharedPtr watchdog_timer_;
    rclcpp::Time  last_command_time_;

    double control_rate_hz_{200.0};
    double state_publish_hz_{200.0};
    double command_watchdog_s_{0.5};
    double watchdog_damping_kd_{2.0};
    bool   watchdog_triggered_{false};
    bool   auto_enable_on_start_{false};
};

}  // namespace w3_robot_bridge
