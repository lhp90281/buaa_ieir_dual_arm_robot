#include "w3_robot_bridge/w3_robot_bridge_node.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <algorithm>
#include <cstdio>
#include <stdexcept>

using namespace std::chrono_literals;

namespace w3_robot_bridge {

W3RobotBridgeNode::W3RobotBridgeNode()
    : rclcpp::Node("w3_robot_bridge_node")
{
    declare_parameters();
    if (!load_config()) {
        RCLCPP_FATAL(get_logger(), "Failed to load configuration");
        throw std::runtime_error("w3_robot_bridge: config load failed");
    }

    if (!bridge_.init(config_path_) || !bridge_.start()) {
        RCLCPP_FATAL(get_logger(), "MotorBridge init/start failed");
        throw std::runtime_error("w3_robot_bridge: bridge init failed");
    }

    setup_ros_interfaces();

    if (auto_enable_on_start_) {
        RCLCPP_INFO(get_logger(), "Auto-enabling all motors...");
        bridge_.enableAll();
    }

    RCLCPP_INFO(get_logger(),
                "W3 Robot Bridge ready: %zu channel(s), %.0f Hz control, %.0f Hz publish",
                bridge_.activeChannels().size(),
                control_rate_hz_,
                state_publish_hz_);
}

W3RobotBridgeNode::~W3RobotBridgeNode() {
    RCLCPP_INFO(get_logger(), "Shutting down W3 Robot Bridge...");
    bridge_.stop();
    bridge_.close();
}

// ── Parameter declaration ──────────────────────────────────────

void W3RobotBridgeNode::declare_parameters() {
    declare_parameter<std::string>("config_path", "");
    declare_parameter<double>("control_rate_hz", 200.0);
    declare_parameter<double>("state_publish_hz", 200.0);
    declare_parameter<double>("feedback_timeout_s", 0.02);
    declare_parameter<double>("command_watchdog_s", 0.5);
    declare_parameter<double>("watchdog_damping_kd", 2.0);
    declare_parameter<bool>("auto_enable_on_start", false);
}

// ── Config loading ─────────────────────────────────────────────

bool W3RobotBridgeNode::load_config() {
    control_rate_hz_       = get_parameter("control_rate_hz").as_double();
    state_publish_hz_      = get_parameter("state_publish_hz").as_double();
    command_watchdog_s_    = get_parameter("command_watchdog_s").as_double();
    watchdog_damping_kd_   = get_parameter("watchdog_damping_kd").as_double();
    auto_enable_on_start_  = get_parameter("auto_enable_on_start").as_bool();

    config_path_ = get_parameter("config_path").as_string();
    if (config_path_.empty()) {
        // Default: package share directory + config/w3_robot_bridge.yaml
        try {
            config_path_ = ament_index_cpp::get_package_share_directory("w3_robot_bridge")
                           + "/config/w3_robot_bridge.yaml";
        } catch (...) {
            RCLCPP_ERROR(get_logger(), "Cannot find package share directory");
            return false;
        }
    }

    RCLCPP_INFO(get_logger(), "Loading config from: %s", config_path_.c_str());
    return true;
}

// ── ROS interface setup ────────────────────────────────────────

void W3RobotBridgeNode::setup_ros_interfaces() {
    using std::placeholders::_1;
    using std::placeholders::_2;

    // Subscribers
    cmd_sub_ = create_subscription<w3_robot_bridge::msg::MotorCommand>(
        "~/command",
        rclcpp::QoS(100).best_effort(),
        std::bind(&W3RobotBridgeNode::on_single_command, this, _1));

    cmds_sub_ = create_subscription<w3_robot_bridge::msg::MotorCommandArray>(
        "~/commands",
        rclcpp::QoS(50).best_effort(),
        std::bind(&W3RobotBridgeNode::on_batch_command, this, _1));

    // Publisher
    state_pub_ = create_publisher<w3_robot_bridge::msg::MotorStateArray>(
        "~/state",
        rclcpp::QoS(50).best_effort());

    // Services
    enable_srv_ = create_service<w3_robot_bridge::srv::EnableMotor>(
        "~/enable",
        std::bind(&W3RobotBridgeNode::on_enable, this, _1, _2));
    disable_srv_ = create_service<w3_robot_bridge::srv::DisableMotor>(
        "~/disable",
        std::bind(&W3RobotBridgeNode::on_disable, this, _1, _2));
    zero_srv_ = create_service<w3_robot_bridge::srv::SetZero>(
        "~/zero",
        std::bind(&W3RobotBridgeNode::on_set_zero, this, _1, _2));

    // State publish timer
    const auto pub_period = std::chrono::duration<double>(
        state_publish_hz_ > 0.0 ? 1.0 / state_publish_hz_ : 0.01);
    state_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(pub_period),
        std::bind(&W3RobotBridgeNode::publish_states, this));

    // Watchdog timer (50ms intervals)
    watchdog_timer_ = create_wall_timer(
        50ms,
        std::bind(&W3RobotBridgeNode::watchdog_tick, this));
    last_command_time_ = get_clock()->now();
}

// ── Command callbacks ──────────────────────────────────────────

void W3RobotBridgeNode::on_single_command(
    const w3_robot_bridge::msg::MotorCommand::SharedPtr msg)
{
    if (!msg) return;
    last_command_time_ = get_clock()->now();

    const uint8_t ch  = msg->channel;
    const uint8_t idx = msg->motor_index;

    // ── Tigertoe (钛虎) modes ──────────────────────────────
    if (ch == 2) {
        auto* tt = bridge_.getTigertoeChannel();
        if (!tt) return;

        switch (msg->mode) {
            case w3_robot_bridge::msg::MotorCommand::MODE_BRAKE:
                tt->sendBrake(idx);
                return;
            case w3_robot_bridge::msg::MotorCommand::MODE_GET_POSITION:
                tt->sendGetPosition(idx);
                return;
            case w3_robot_bridge::msg::MotorCommand::MODE_SET_POSITION: {
                const auto& pro = tt->profiles()[idx];
                // Clamp to motor position limits
                float clamped = std::max(pro.position_min,
                    std::min(pro.position_max, msg->position));
                int32_t cnt = tigertoe::rad_to_cnt(clamped,
                    pro.zero_offset, pro.gear_ratio, pro.direction);
                tt->sendSetPosition(idx, cnt);
                return;
            }
            default:
                return;
        }
    }

    // ── Damiao modes ─────────────────────────────────────
    // Handle mode transitions
    switch (msg->mode) {
        case w3_robot_bridge::msg::MotorCommand::MODE_ENABLE:
            bridge_.enableMotor(ch, idx);
            return;
        case w3_robot_bridge::msg::MotorCommand::MODE_DISABLE:
            bridge_.disableMotor(ch, idx);
            return;
        case w3_robot_bridge::msg::MotorCommand::MODE_ZERO:
            bridge_.setZero(ch, idx);
            return;
        case w3_robot_bridge::msg::MotorCommand::MODE_RUN:
        default:
            break;
    }

    // MODE_RUN: forward as RUN command
    MotorCommand cmd{MotorMode::RUN,
                     msg->position,
                     msg->velocity,
                     msg->kp,
                     msg->kd,
                     msg->torque};
    bridge_.setCommand(ch, idx, cmd);
}

void W3RobotBridgeNode::on_batch_command(
    const w3_robot_bridge::msg::MotorCommandArray::SharedPtr msg)
{
    if (!msg) return;
    last_command_time_ = get_clock()->now();

    // Efficient batch dispatch: iterate and forward each command.
    // The MotorCommandArray allows controlling multiple motors with a
    // single ROS message, avoiding the overhead of many individual
    // topic publications.
    for (const auto& c : msg->commands) {
        on_single_command(
            std::make_shared<w3_robot_bridge::msg::MotorCommand>(c));
    }
}

// ── Service callbacks ──────────────────────────────────────────

void W3RobotBridgeNode::on_enable(
    const w3_robot_bridge::srv::EnableMotor::Request::SharedPtr req,
    w3_robot_bridge::srv::EnableMotor::Response::SharedPtr res)
{
    if (req->channel == 255) {
        bridge_.enableAll();
    } else if (req->motor_index == 255) {
        for (uint8_t i = 0; i < bridge_.motorCount(req->channel); ++i) {
            bridge_.enableMotor(req->channel, i);
        }
    } else {
        bridge_.enableMotor(req->channel, req->motor_index);
    }
    res->success = true;
    res->message = "ok";
}

void W3RobotBridgeNode::on_disable(
    const w3_robot_bridge::srv::DisableMotor::Request::SharedPtr req,
    w3_robot_bridge::srv::DisableMotor::Response::SharedPtr res)
{
    if (req->channel == 255) {
        bridge_.disableAll();
    } else if (req->motor_index == 255) {
        for (uint8_t i = 0; i < bridge_.motorCount(req->channel); ++i) {
            bridge_.disableMotor(req->channel, i);
        }
    } else {
        bridge_.disableMotor(req->channel, req->motor_index);
    }
    res->success = true;
    res->message = "ok";
}

void W3RobotBridgeNode::on_set_zero(
    const w3_robot_bridge::srv::SetZero::Request::SharedPtr req,
    w3_robot_bridge::srv::SetZero::Response::SharedPtr res)
{
    bridge_.setZero(req->channel, req->motor_index);
    res->success = true;
    res->message = "zero command sent";
}

// ── State publishing ───────────────────────────────────────────

void W3RobotBridgeNode::publish_states() {
    auto msg = w3_robot_bridge::msg::MotorStateArray();
    const auto stamp = get_clock()->now();
    msg.header.stamp = stamp;
    msg.header.frame_id = "w3_robot_bridge";

    for (uint8_t ch : bridge_.activeChannels()) {
        const uint8_t n_motors = bridge_.motorCount(ch);
        for (uint8_t i = 0; i < n_motors; ++i) {
            const auto fb = bridge_.getState(ch, i);
            const auto* prof = bridge_.getProfile(ch, i);

            w3_robot_bridge::msg::MotorState m;
            m.header.stamp    = stamp;
            m.header.frame_id = "w3_robot_bridge";
            m.channel         = ch;
            m.motor_index     = i;
            m.can_id          = prof ? prof->can_id : fb.can_id;
            m.position        = fb.position;
            m.velocity        = fb.velocity;
            m.torque          = fb.torque;
            m.error_flags     = fb.error_flags;
            m.enabled         = fb.enabled;
            m.online          = fb.online;
            msg.motors.push_back(m);
        }
    }

    // ── Tigertoe states ──────────────────────────────────────
    if (bridge_.hasTigertoeChannel()) {
        auto* tt = bridge_.getTigertoeChannel();
        tt->refreshOnlineFlags();
        for (uint8_t i = 0; i < tt->motorCount(); ++i) {
            const auto fb = tt->getState(i);
            const auto& pro = tt->profiles()[i];

            w3_robot_bridge::msg::MotorState m;
            m.header.stamp    = stamp;
            m.header.frame_id = "w3_robot_bridge";
            m.channel         = 2;
            m.motor_index     = i;
            m.can_id          = pro.can_id;
            m.position        = fb.position_rad;
            m.velocity        = 0.0f;
            m.torque          = 0.0f;
            m.error_flags     = 0;
            m.enabled         = true;     // tigertoe motors always "enabled" (no enable protocol)
            m.online          = fb.online;
            msg.motors.push_back(m);
        }
    }

    state_pub_->publish(std::move(msg));
}

// ── Watchdog ───────────────────────────────────────────────────

void W3RobotBridgeNode::watchdog_tick() {
    const auto now_tp = get_clock()->now();
    const double dt   = (now_tp - last_command_time_).seconds();
    if (dt <= command_watchdog_s_) {
        watchdog_triggered_ = false;  // reset flag when commands resume
        return;
    }

    // Already in damping state — no need to keep re-sending
    if (watchdog_triggered_) return;
    watchdog_triggered_ = true;

    // Safety: send damping brake to all motors.
    // DAMP mode: kp=0, kd=watchdog_damping_kd → pure velocity damping.
    // motor_current = -KD * velocity / KT → resists motion proportionally
    // to speed, acting as a soft brake that protects the gear train.
    MotorCommand damp_cmd{
        MotorMode::DAMP,
        0.0f, 0.0f,                               // position, velocity = 0
        0.0f,                                     // kp = 0 (no position hold)
        static_cast<float>(watchdog_damping_kd_), // kd > 0 (damping)
        0.0f                                      // torque = 0
    };

    RCLCPP_WARN(get_logger(),
                "Watchdog triggered (%.1fs no command)! "
                "Sending damping brake (kd=%.1f) to all motors.",
                dt, watchdog_damping_kd_);

    for (uint8_t ch : bridge_.activeChannels()) {
        const uint8_t n = bridge_.motorCount(ch);
        for (uint8_t i = 0; i < n; ++i) {
            bridge_.setCommand(ch, i, damp_cmd);
        }
    }

    // ── Tigertoe watchdog: send brake ────────────────────────
    if (bridge_.hasTigertoeChannel()) {
        auto* tt = bridge_.getTigertoeChannel();
        for (uint8_t i = 0; i < tt->motorCount(); ++i) {
            tt->sendBrake(i);
        }
    }
}

}  // namespace w3_robot_bridge

// ── main ──────────────────────────────────────────────────────
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<w3_robot_bridge::W3RobotBridgeNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "[w3_bridge] Fatal: %s\n", e.what());
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
