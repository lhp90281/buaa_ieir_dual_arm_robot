// ============================================================================
// eiriarm_controllers/gripper_controller_node.cpp
//
// Standalone gripper controller for two DM4310-driven parallel grippers wired
// to W3 bridge can0.slot7 (left) and can1.slot7 (right). Talks directly to the
// W3 bridge via /w3_robot_bridge_node/commands and /w3_robot_bridge_node/state
// (MotorStateArray); intentionally OUTSIDE ros2_control so the gripper can
// start/stop independently of the arm controllers (see comment in
// dual_arm_ros2_control.urdf.xacro for the policy).
//
// State machine per gripper:
//   INIT             -- waiting for first MotorState frame
//   CALIBRATING_OPEN -- low-speed push toward OPEN, stall = end-of-stroke
//   CALIBRATING_CLOSE-- low-speed push toward CLOSE, stall = end-of-stroke
//   MOVING_OPEN      -- user-commanded open; stops on limit or force
//   MOVING_CLOSE     -- user-commanded close; stops on force_threshold ->
//                       FORCE_HOLD, or on close limit -> HOLD
//   HOLD             -- locked at q_held_motor via motor PD
//   FORCE_HOLD       -- maintained grip at q_contact + close_dir*overshoot
//                       with kp tuned so steady-state torque ~= force_thr
//   TELEOP_PASSIVE   -- master-side teleop: zero stiffness, friction-only
//                       feed-forward from measured velocity so fingers can
//                       back-drive the gripper.
//   TELEOP_TRACK     -- slave-side teleop: track normalized opening ratio
//                       from the remote master gripper.
//   FAILED           -- calibration timed out; controller refuses to move
//
// Multi-turn tracking:
//   DM4310 reports raw_pos in [-pos_max, +pos_max] rad (single-turn window).
//   We unwrap delta into the same window and accumulate into q_motor (a
//   free-running multi-turn motor angle). Stroke can exceed the single-turn
//   window without losing track.
//
// Friction compensation (DM4310 row of friction_model.yaml, same file gravity
// controller uses):
//   tau_ff = friction_gain * (sign(v)*coulomb_dir + viscous*v)  for |v|>deadband
//
// Command grammar (std_msgs/String on ~/{left,right}_gripper/command):
//   "open"               -- open at default speed, no force gate
//   "close"              -- close, stop when |torque| > default_force_thr
//   "close 2.0"          -- close, override force_thr = 2.0 Nm
//   "close 2.0 1.5"      -- also override speed = 1.5 rad/s
//   "hold"               -- lock at current position
//   "calibrate"          -- redo open->close calibration cycle
//
// Status diagnostic on ~/{left,right}_gripper/status (std_msgs/String).
// ============================================================================

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <w3_robot_bridge/msg/motor_command_array.hpp>
#include <w3_robot_bridge/msg/motor_state_array.hpp>

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdint>
#include <fstream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace eiriarm_controllers
{

enum class GState : uint8_t {
  INIT,
  ENABLING,           // FC sent; waiting for DM motor to come up
  CALIBRATING_OPEN,
  CALIBRATING_CLOSE,
  MOVING_OPEN,
  MOVING_CLOSE,
  HOLD,
  FORCE_HOLD,
  TELEOP_PASSIVE,
  TELEOP_TRACK,
  FAILED,
};

static const char * state_str(GState s)
{
  switch (s) {
    case GState::INIT:              return "init";
    case GState::ENABLING:          return "enabling";
    case GState::CALIBRATING_OPEN:  return "cal_open";
    case GState::CALIBRATING_CLOSE: return "cal_close";
    case GState::MOVING_OPEN:       return "moving_open";
    case GState::MOVING_CLOSE:      return "moving_close";
    case GState::HOLD:              return "hold";
    case GState::FORCE_HOLD:        return "force_hold";
    case GState::TELEOP_PASSIVE:    return "teleop_passive";
    case GState::TELEOP_TRACK:      return "teleop_track";
    case GState::FAILED:            return "failed";
  }
  return "?";
}

struct Gripper
{
  // identity
  std::string side;        // "left" / "right" (logging only)
  uint8_t channel = 0;     // 0 = left/can0, 1 = right/can1
  uint8_t slot = 7;
  bool enabled = false;

  // motor params (DM4310 unique-decoding window etc.)
  double pos_max = 12.5;
  double vel_max = 50.0;
  double tor_max = 10.0;

  // friction model (DM4310 row)
  bool   fric_enabled = false;
  double fric_viscous = 0.0;
  double fric_coulomb_pos = 0.0;
  double fric_coulomb_neg = 0.0;
  double fric_gain = 0.5;
  double fric_deadband = 0.05;     // rad/s
  double velocity_filter_alpha = 0.9;

  // mechanical wiring: which sign of motor radians = "closing"
  int close_motor_direction = +1;  // +1 or -1

  // calibration tunables
  double cal_speed = 2.0;          // rad/s motor
  double cal_kp = 3.0;             // Nm/rad
  double cal_kd = 0.5;             // Nm/(rad/s)
  double cal_stall_torque = 1.5;   // Nm
  double cal_stall_vel_thr = 0.2;  // rad/s (must be near zero)
  int    cal_stall_ticks = 50;     // # consecutive frames above stall before commit
  int    cal_phase_timeout_ticks = 0;  // filled from cal_phase_timeout_s * rate

  // motion / hold tunables
  double move_speed = 3.0;         // rad/s default
  double move_kp = 5.0;
  double move_kd = 0.5;
  double hold_kp = 8.0;
  double hold_kd = 0.4;
  double force_threshold = 1.0;    // Nm default
  int    force_grace_ticks = 30;   // ignore force this many frames after move start
  double force_hold_overshoot = 0.1;  // rad; kp = thr/overshoot in FORCE_HOLD
  double teleop_track_kp = 5.0;
  double teleop_track_kd = 0.5;
  double teleop_track_max_step = 0.20;  // motor rad/tick
  double teleop_passive_kd = 0.0;

  // ---- runtime state (raw motor frame) ----
  bool   has_state = false;
  double raw_pos = 0.0;
  double raw_pos_prev = 0.0;
  double velocity = 0.0;
  double velocity_filtered = 0.0;
  double torque = 0.0;
  uint8_t err = 0;
  double q_motor = 0.0;            // multi-turn accumulator
  rclcpp::Time last_state_stamp;

  // ---- runtime state machine ----
  GState state = GState::INIT;
  int stall_counter = 0;
  int phase_ticks = 0;             // ticks in current state (for grace / timeout)
  int enable_confirm_count = 0;    // consecutive frames with err == DM_ERR_ENABLED

  // ENABLING tunables (filled from params at setup). Mirrors what
  // dm_hardware_interface does for the arm motors:
  //   - publish FC at high rate (50 Hz default) for several seconds
  //   - publish zero MotorCommand every tick to refresh the W3 command heartbeat
  //   - exit when err == 1 for enable_confirm_frames consecutive frames
  int enable_pub_period_ticks = 0;       // ticks between FC publishes
  int enable_timeout_ticks = 0;          // hard timeout for the whole phase
  int enable_confirm_frames = 5;         // err==1 streak required to commit

  // calibration results
  bool   calibrated = false;
  double q_open_motor = 0.0;
  double q_close_motor = 0.0;
  double stroke_motor = 0.0;       // |q_close - q_open|

  // active motion / hold targets
  double q_target = 0.0;           // for MOVING_*, integrated step target
  double q_held  = 0.0;            // for HOLD / FORCE_HOLD anchor
  double q_teleop_target = 0.0;    // for TELEOP_TRACK
  double teleop_target_ratio = 0.0;
  double cmd_speed = 0.0;          // signed rad/s of current motion
  double cmd_force_thr = 0.0;      // Nm threshold of current close

  // ---- arm-readiness gating (per-channel mirror) ----
  // Counts how many of slots 0..6 on this gripper's CAN channel report
  // err == DM_ERR_ENABLED (=1). The arm hw_interface owns slots 0..6 so
  // this is a clean signal that "the arm on my channel is up".
  // Updated on every W3 state frame.
  int  arm_armed_count = 0;        // 0..7
  bool arm_was_ever_ready = false; // latched only while the arm remains ready
};

class GripperController : public rclcpp::Node
{
public:
  GripperController()
  : Node("gripper_controller")
  {
    // ---- generic params ----
    declare_parameter<double>("control_rate", 500.0);
    declare_parameter<std::string>("friction_model_yaml", "");
    declare_parameter<bool>("friction_compensation_enabled", false);
    declare_parameter<double>("friction_gain", 0.5);
    declare_parameter<double>("friction_deadband", 0.05);
    declare_parameter<double>("velocity_filter_alpha", 0.9);

    // ---- per-arm enable / wiring ----
    declare_parameter<bool>("left_enabled",  true);
    declare_parameter<bool>("right_enabled", true);

    // ---- per-arm tuning (mirrored defaults). Keep these flat so a YAML
    //      with `left_*` / `right_*` keys can override either side. ----
    declare_default_gripper_params("left");
    declare_default_gripper_params("right");

    const double rate = get_parameter("control_rate").as_double();
    dt_ = 1.0 / std::max(1.0, rate);

    setup_gripper(left_,  "left",  /*channel=*/0);
    setup_gripper(right_, "right", /*channel=*/1);

    // Friction model loaded once (DM4310 row applied to both grippers).
    const bool fric_enable = get_parameter("friction_compensation_enabled").as_bool();
    const std::string fric_yaml = get_parameter("friction_model_yaml").as_string();
    if (fric_enable) {
      if (!load_dm4310_friction(fric_yaml)) {
        RCLCPP_WARN(get_logger(),
          "Friction compensation requested but failed to load '%s'; disabling.",
          fric_yaml.c_str());
        left_.fric_enabled = false;
        right_.fric_enabled = false;
      }
    }

    // ---- ROS interfaces ----
    rclcpp::QoS state_qos(rclcpp::KeepLast(10));
    state_qos.best_effort();  // matches W3 publisher QoS

    if (left_.enabled) {
      if (!state_sub_) {
        state_sub_ = create_subscription<w3_robot_bridge::msg::MotorStateArray>(
          "/w3_robot_bridge_node/state", state_qos,
          [this](w3_robot_bridge::msg::MotorStateArray::SharedPtr m) {
            on_state(left_, *m);
            on_state(right_, *m);
          });
      }
      if (!cmd_pub_) {
        rclcpp::QoS cmd_qos(rclcpp::KeepLast(50));
        cmd_qos.best_effort();
        cmd_pub_ = create_publisher<w3_robot_bridge::msg::MotorCommandArray>(
          "/w3_robot_bridge_node/commands", cmd_qos);
      }
      left_cmd_sub_ = create_subscription<std_msgs::msg::String>(
        "~/left_gripper/command", 10,
        [this](std_msgs::msg::String::SharedPtr m) { on_command(left_, m->data); });
      left_teleop_target_sub_ = create_subscription<std_msgs::msg::Float32>(
        "~/left_gripper/teleop_target", 10,
        [this](std_msgs::msg::Float32::SharedPtr m) { on_teleop_target(left_, m->data); });
      left_status_pub_ = create_publisher<std_msgs::msg::String>(
        "~/left_gripper/status", 10);
      left_teleop_state_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(
        "~/left_gripper/teleop_state", 10);
    }
    if (right_.enabled) {
      if (!state_sub_) {
        state_sub_ = create_subscription<w3_robot_bridge::msg::MotorStateArray>(
          "/w3_robot_bridge_node/state", state_qos,
          [this](w3_robot_bridge::msg::MotorStateArray::SharedPtr m) {
            on_state(left_, *m);
            on_state(right_, *m);
          });
      }
      if (!cmd_pub_) {
        rclcpp::QoS cmd_qos(rclcpp::KeepLast(50));
        cmd_qos.best_effort();
        cmd_pub_ = create_publisher<w3_robot_bridge::msg::MotorCommandArray>(
          "/w3_robot_bridge_node/commands", cmd_qos);
      }
      right_cmd_sub_ = create_subscription<std_msgs::msg::String>(
        "~/right_gripper/command", 10,
        [this](std_msgs::msg::String::SharedPtr m) { on_command(right_, m->data); });
      right_teleop_target_sub_ = create_subscription<std_msgs::msg::Float32>(
        "~/right_gripper/teleop_target", 10,
        [this](std_msgs::msg::Float32::SharedPtr m) { on_teleop_target(right_, m->data); });
      right_status_pub_ = create_publisher<std_msgs::msg::String>(
        "~/right_gripper/status", 10);
      right_teleop_state_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(
        "~/right_gripper/teleop_state", 10);
    }

    using namespace std::chrono;
    const auto period = duration_cast<nanoseconds>(duration<double>(dt_));
    control_timer_ = create_wall_timer(period, [this] { tick(); });

    // 1 Hz status publisher (separate from tick to avoid log spam)
    status_timer_ = create_wall_timer(1s, [this] {
      publish_status(left_,  left_status_pub_);
      publish_status(right_, right_status_pub_);
    });

    RCLCPP_INFO(get_logger(),
      "GripperController up. rate=%.0f Hz, left=%s right=%s, fric=%s",
      1.0 / dt_, left_.enabled ? "on" : "off", right_.enabled ? "on" : "off",
      left_.fric_enabled || right_.fric_enabled ? "on" : "off");
  }

private:
  // --------------------------------------------------------------------------
  // Parameter scaffolding
  // --------------------------------------------------------------------------
  void declare_default_gripper_params(const std::string & p)
  {
    declare_parameter<int>   (p + "_close_motor_direction", +1);

    declare_parameter<double>(p + "_pos_max", 12.5);
    declare_parameter<double>(p + "_vel_max", 50.0);
    declare_parameter<double>(p + "_tor_max", 10.0);

    declare_parameter<double>(p + "_cal_speed", 2.0);
    declare_parameter<double>(p + "_cal_kp",    3.0);
    declare_parameter<double>(p + "_cal_kd",    0.5);
    declare_parameter<double>(p + "_cal_stall_torque", 1.5);
    declare_parameter<double>(p + "_cal_stall_vel_thr", 0.2);
    declare_parameter<int>   (p + "_cal_stall_ticks", 50);
    declare_parameter<double>(p + "_cal_phase_timeout_s", 15.0);

    declare_parameter<double>(p + "_move_speed",   3.0);
    declare_parameter<double>(p + "_move_kp",      5.0);
    declare_parameter<double>(p + "_move_kd",      0.5);
    declare_parameter<double>(p + "_hold_kp",      8.0);
    declare_parameter<double>(p + "_hold_kd",      0.4);
    declare_parameter<double>(p + "_force_threshold", 1.0);
    declare_parameter<int>   (p + "_force_grace_ticks", 30);
    declare_parameter<double>(p + "_force_hold_overshoot", 0.1);
    declare_parameter<double>(p + "_teleop_track_kp", 5.0);
    declare_parameter<double>(p + "_teleop_track_kd", 0.5);
    declare_parameter<double>(p + "_teleop_track_max_step", 0.20);
    declare_parameter<double>(p + "_teleop_passive_kd", 0.0);

    // ENABLING phase: matches dm_hardware_interface's strategy.
    //   _enable_pub_period_s : how often to re-publish FC (50 Hz default).
    //   _enable_timeout_s    : whole-phase timeout before going to FAILED.
    //   _enable_confirm_frames : require err==1 for this many *consecutive*
    //                            state frames before declaring enabled.
    declare_parameter<double>(p + "_enable_pub_period_s",  0.02);
    declare_parameter<double>(p + "_enable_timeout_s",     5.0);
    declare_parameter<int>   (p + "_enable_confirm_frames", 5);
  }

  void setup_gripper(Gripper & g, const std::string & side, uint8_t channel)
  {
    g.side = side;
    g.channel = channel;
    g.slot = 7;
    g.enabled = get_parameter(side + "_enabled").as_bool();
    if (!g.enabled) {
      return;
    }

    g.close_motor_direction = get_parameter(side + "_close_motor_direction").as_int();
    if (g.close_motor_direction != +1 && g.close_motor_direction != -1) {
      RCLCPP_WARN(get_logger(), "%s_close_motor_direction must be +1 or -1; got %d -> using +1",
        side.c_str(), g.close_motor_direction);
      g.close_motor_direction = +1;
    }

    g.pos_max = get_parameter(side + "_pos_max").as_double();
    g.vel_max = get_parameter(side + "_vel_max").as_double();
    g.tor_max = get_parameter(side + "_tor_max").as_double();

    g.cal_speed = get_parameter(side + "_cal_speed").as_double();
    g.cal_kp    = get_parameter(side + "_cal_kp").as_double();
    g.cal_kd    = get_parameter(side + "_cal_kd").as_double();
    g.cal_stall_torque  = get_parameter(side + "_cal_stall_torque").as_double();
    g.cal_stall_vel_thr = get_parameter(side + "_cal_stall_vel_thr").as_double();
    g.cal_stall_ticks   = static_cast<int>(get_parameter(side + "_cal_stall_ticks").as_int());

    const double t_s = get_parameter(side + "_cal_phase_timeout_s").as_double();
    g.cal_phase_timeout_ticks = static_cast<int>(t_s / dt_);

    g.move_speed = get_parameter(side + "_move_speed").as_double();
    g.move_kp    = get_parameter(side + "_move_kp").as_double();
    g.move_kd    = get_parameter(side + "_move_kd").as_double();
    g.hold_kp    = get_parameter(side + "_hold_kp").as_double();
    g.hold_kd    = get_parameter(side + "_hold_kd").as_double();
    g.force_threshold = get_parameter(side + "_force_threshold").as_double();
    g.force_grace_ticks = static_cast<int>(get_parameter(side + "_force_grace_ticks").as_int());
    g.force_hold_overshoot = get_parameter(side + "_force_hold_overshoot").as_double();
    g.teleop_track_kp = get_parameter(side + "_teleop_track_kp").as_double();
    g.teleop_track_kd = get_parameter(side + "_teleop_track_kd").as_double();
    g.teleop_track_max_step = get_parameter(side + "_teleop_track_max_step").as_double();
    g.teleop_passive_kd = get_parameter(side + "_teleop_passive_kd").as_double();

    g.fric_gain     = get_parameter("friction_gain").as_double();
    g.fric_deadband = get_parameter("friction_deadband").as_double();
    g.velocity_filter_alpha = std::clamp(
      get_parameter("velocity_filter_alpha").as_double(), 0.0, 0.999);

    const double pub_p_s = get_parameter(side + "_enable_pub_period_s").as_double();
    const double tout_s  = get_parameter(side + "_enable_timeout_s").as_double();
    g.enable_pub_period_ticks = std::max(1, static_cast<int>(pub_p_s / dt_));
    g.enable_timeout_ticks    = std::max(1, static_cast<int>(tout_s / dt_));
    g.enable_confirm_frames   =
      std::max(1, static_cast<int>(get_parameter(side + "_enable_confirm_frames").as_int()));
  }

  // --------------------------------------------------------------------------
  // Friction model loader -- mirrors gravity_compensation_controller pattern.
  // Only the DM4310 row is consumed since both grippers are DM4310.
  // --------------------------------------------------------------------------
  bool load_dm4310_friction(const std::string & yaml_path)
  {
    if (yaml_path.empty()) {
      RCLCPP_ERROR(get_logger(),
        "friction_compensation_enabled but friction_model_yaml is empty");
      return false;
    }
    std::ifstream f(yaml_path);
    if (!f.good()) {
      RCLCPP_ERROR(get_logger(),
        "Cannot open friction_model_yaml '%s'", yaml_path.c_str());
      return false;
    }
    YAML::Node root;
    try {
      root = YAML::LoadFile(yaml_path);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(),
        "Failed to parse friction_model_yaml '%s': %s",
        yaml_path.c_str(), e.what());
      return false;
    }
    if (!root["DM4310"]) {
      RCLCPP_ERROR(get_logger(),
        "friction_model_yaml '%s' has no DM4310 entry", yaml_path.c_str());
      return false;
    }
    const auto & m = root["DM4310"];
    try {
      const double viscous     = m["viscous"].as<double>();
      const double coulomb_pos = m["coulomb_pos"].as<double>();
      const double coulomb_neg = m["coulomb_neg"].as<double>();
      auto apply = [&](Gripper & g) {
        if (!g.enabled) {
          return;
        }
        g.fric_enabled = true;
        g.fric_viscous = viscous;
        g.fric_coulomb_pos = coulomb_pos;
        g.fric_coulomb_neg = coulomb_neg;
        RCLCPP_INFO(get_logger(),
          "Friction[%s gripper -> DM4310]: gain=%.3f viscous=%.4f "
          "coulomb_pos=%.3f coulomb_neg=%.3f deadband=%.3f",
          g.side.c_str(), g.fric_gain, g.fric_viscous,
          g.fric_coulomb_pos, g.fric_coulomb_neg, g.fric_deadband);
      };
      apply(left_);
      apply(right_);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(),
        "DM4310 entry missing/invalid fields: %s", e.what());
      return false;
    }
    return true;
  }

  // --------------------------------------------------------------------------
  // Sensor callback: pick out slot 7 from the MotorStateArray and update the
  // gripper's raw_pos / velocity / torque + multi-turn unwrap.
  // --------------------------------------------------------------------------
  void on_state(Gripper & g, const w3_robot_bridge::msg::MotorStateArray & msg)
  {
    if (!g.enabled) {
      return;
    }
    const w3_robot_bridge::msg::MotorState * ms = nullptr;
    for (const auto & motor : msg.motors) {
      if (motor.channel == g.channel && motor.motor_index == g.slot) {
        ms = &motor;
        break;
      }
    }
    if (!ms) {
      return;
    }

    std::lock_guard<std::mutex> lock(mu_);

    // Count arm slots 0..6 that report enabled=true on the
    // SAME CAN channel as this gripper. This is a free side-effect of every
    // state msg and gives us a clean "arm is up" signal without subscribing
    // to /joint_states (which would couple us to the controller_manager
    // node lifecycle).
    int armed = 0;
    for (const auto & motor : msg.motors) {
      if (motor.channel == g.channel && motor.motor_index < 7 &&
          motor.online && motor.enabled) {
        ++armed;
      }
    }
    g.arm_armed_count = armed;
    if (armed == 7) {
      if (!g.arm_was_ever_ready) {
        RCLCPP_INFO(get_logger(),
          "%s gripper: arm on ch%u reports all 7 joints ENABLED -> "
          "gating released, gripper may now ENABLE itself.",
          g.side.c_str(), static_cast<unsigned>(g.channel));
      }
      g.arm_was_ever_ready = true;
    } else if (armed == 0) {
      g.arm_was_ever_ready = false;
    }

    if (!ms->online || !std::isfinite(ms->position) ||
        !std::isfinite(ms->velocity) || !std::isfinite(ms->torque)) {
      return;
    }
    const double now_pos = static_cast<double>(ms->position);
    if (!g.has_state) {
      g.raw_pos_prev = now_pos;
      g.q_motor = now_pos;  // arbitrary origin; calibration writes the real one
      g.q_target = g.q_motor;
      g.q_held = g.q_motor;
      g.has_state = true;
      RCLCPP_INFO(get_logger(),
        "%s gripper: first state frame received (raw_pos=%.3f, err=%u, |tau|=%.2fNm)",
        g.side.c_str(), now_pos, static_cast<unsigned>(ms->error_flags),
        std::abs(static_cast<double>(ms->torque)));
    } else {
      g.q_motor += unwrap_delta(now_pos, g.raw_pos_prev, g.pos_max);
    }
    g.raw_pos = now_pos;
    g.raw_pos_prev = now_pos;
    g.velocity = static_cast<double>(ms->velocity);
    g.velocity_filtered =
      (1.0 - g.velocity_filter_alpha) * g.velocity +
      g.velocity_filter_alpha * g.velocity_filtered;
    g.torque   = static_cast<double>(ms->torque);
    g.err      = ms->error_flags;
    g.last_state_stamp = msg.header.stamp;
  }

  static double unwrap_delta(double now, double prev, double pos_max)
  {
    double d = now - prev;
    const double window = 2.0 * pos_max;
    while (d >  pos_max) {
      d -= window;
    }
    while (d < -pos_max) {
      d += window;
    }
    return d;
  }

  static double gripper_ratio(const Gripper & g)
  {
    if (!g.calibrated) {
      return 0.0;
    }
    const double denom = g.q_close_motor - g.q_open_motor;
    if (std::abs(denom) < 1e-6) {
      return 0.0;
    }
    return std::clamp((g.q_motor - g.q_open_motor) / denom, 0.0, 1.0);
  }

  static double ratio_to_motor(const Gripper & g, double ratio)
  {
    const double r = std::clamp(ratio, 0.0, 1.0);
    return g.q_open_motor + r * (g.q_close_motor - g.q_open_motor);
  }

  void on_teleop_target(Gripper & g, float ratio)
  {
    if (!g.enabled) {
      return;
    }
    std::lock_guard<std::mutex> lock(mu_);
    if (!g.calibrated) {
      return;
    }
    g.teleop_target_ratio = std::clamp(static_cast<double>(ratio), 0.0, 1.0);
    if (g.state == GState::TELEOP_TRACK) {
      // The control loop will slew-limit q_teleop_target toward this ratio.
      return;
    }
  }

  // --------------------------------------------------------------------------
  // User command parser. Grammar:
  //   open
  //   close [force_thr_Nm [speed_rad_s]]
  //   hold
  //   teleop_passive
  //   teleop_track [ratio_0_to_1]
  //   calibrate
  // --------------------------------------------------------------------------
  void on_command(Gripper & g, const std::string & raw)
  {
    if (!g.enabled) {
      return;
    }
    std::istringstream iss(raw);
    std::string verb;
    iss >> verb;
    std::transform(verb.begin(), verb.end(), verb.begin(),
                   [](unsigned char c) { return std::tolower(c); });

    std::lock_guard<std::mutex> lock(mu_);

    if (g.state == GState::FAILED) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "%s gripper FAILED; ignoring '%s'. Send 'calibrate' to retry.",
        g.side.c_str(), verb.c_str());
      if (verb != "calibrate") {
        return;
      }
    }
    if (is_calibrating(g.state) && verb != "calibrate") {
      RCLCPP_WARN(get_logger(),
        "%s gripper is calibrating; ignoring '%s'", g.side.c_str(), verb.c_str());
      return;
    }

    if (verb == "open") {
      if (!g.calibrated) {
        RCLCPP_WARN(get_logger(),
          "%s gripper not calibrated; ignoring 'open'", g.side.c_str());
        return;
      }
      g.cmd_speed = -g.close_motor_direction * std::abs(g.move_speed);
      g.q_target  = g.q_motor;
      g.state = GState::MOVING_OPEN;
      g.phase_ticks = 0;
      RCLCPP_INFO(get_logger(), "%s gripper -> MOVING_OPEN @%.2f rad/s",
        g.side.c_str(), g.cmd_speed);
    } else if (verb == "close") {
      if (!g.calibrated) {
        RCLCPP_WARN(get_logger(),
          "%s gripper not calibrated; ignoring 'close'", g.side.c_str());
        return;
      }
      double thr = g.force_threshold;
      double spd = g.move_speed;
      iss >> thr;  // ignore parse failures; iss keeps thr unchanged
      iss >> spd;
      g.cmd_force_thr = std::abs(thr);
      g.cmd_speed = g.close_motor_direction * std::abs(spd);
      g.q_target  = g.q_motor;
      g.state = GState::MOVING_CLOSE;
      g.phase_ticks = 0;
      RCLCPP_INFO(get_logger(),
        "%s gripper -> MOVING_CLOSE thr=%.2fNm spd=%.2frad/s",
        g.side.c_str(), g.cmd_force_thr, g.cmd_speed);
    } else if (verb == "hold") {
      g.q_held = g.q_motor;
      g.state = GState::HOLD;
      g.phase_ticks = 0;
      RCLCPP_INFO(get_logger(), "%s gripper -> HOLD @%.3frad", g.side.c_str(), g.q_held);
    } else if (verb == "teleop_passive") {
      if (!g.calibrated) {
        RCLCPP_WARN(get_logger(),
          "%s gripper not calibrated; ignoring 'teleop_passive'", g.side.c_str());
        return;
      }
      g.state = GState::TELEOP_PASSIVE;
      g.phase_ticks = 0;
      RCLCPP_INFO(get_logger(),
        "%s gripper -> TELEOP_PASSIVE (kp=0, friction from measured velocity)",
        g.side.c_str());
    } else if (verb == "teleop_track") {
      if (!g.calibrated) {
        RCLCPP_WARN(get_logger(),
          "%s gripper not calibrated; ignoring 'teleop_track'", g.side.c_str());
        return;
      }
      double ratio = gripper_ratio(g);
      iss >> ratio;
      g.teleop_target_ratio = std::clamp(ratio, 0.0, 1.0);
      g.q_teleop_target = ratio_to_motor(g, g.teleop_target_ratio);
      g.state = GState::TELEOP_TRACK;
      g.phase_ticks = 0;
      RCLCPP_INFO(get_logger(),
        "%s gripper -> TELEOP_TRACK ratio=%.3f q=%.3frad",
        g.side.c_str(), g.teleop_target_ratio, g.q_teleop_target);
    } else if (verb == "calibrate") {
      enter_enabling(g);
    } else {
      RCLCPP_WARN(get_logger(),
        "%s gripper: unknown command '%s' "
        "(use open/close/hold/teleop_passive/teleop_track/calibrate)",
        g.side.c_str(), verb.c_str());
    }
  }

  static bool is_calibrating(GState s)
  {
    return s == GState::ENABLING ||
           s == GState::CALIBRATING_OPEN ||
           s == GState::CALIBRATING_CLOSE;
  }

  void enter_enabling(Gripper & g)
  {
    g.calibrated = false;
    g.enable_confirm_count = 0;
    g.phase_ticks = 0;
    g.stall_counter = 0;
    g.state = GState::ENABLING;
    RCLCPP_INFO(get_logger(),
      "%s gripper: entering ENABLING (FC at %.0f Hz + zero-cmd; need "
      "err==1 for %d frames; timeout %.1fs)",
      g.side.c_str(),
      1.0 / (g.enable_pub_period_ticks * dt_),
      g.enable_confirm_frames,
      g.enable_timeout_ticks * dt_);
  }

  void start_calibration(Gripper & g)
  {
    g.calibrated = false;
    g.stall_counter = 0;
    g.phase_ticks = 0;
    g.cmd_speed = -g.close_motor_direction * std::abs(g.cal_speed);  // open first
    g.q_target = g.q_motor;
    g.state = GState::CALIBRATING_OPEN;
    RCLCPP_INFO(get_logger(),
      "%s gripper: starting calibration (open first, then close) @cmd_speed=%.2frad/s",
      g.side.c_str(), g.cmd_speed);
  }

  // --------------------------------------------------------------------------
  // Control loop tick at control_rate Hz.
  // --------------------------------------------------------------------------
  void tick()
  {
    std::lock_guard<std::mutex> lock(mu_);
    if (left_.enabled) {
      step(left_,  cmd_pub_);
    }
    if (right_.enabled) {
      step(right_, cmd_pub_);
    }
  }

  void step(Gripper & g,
            const rclcpp::Publisher<w3_robot_bridge::msg::MotorCommandArray>::SharedPtr & pub)
  {
    if (shutdown_in_progress_.load()) {
      // Shutdown sequence is running -- it owns the FD path. Stop publishing
      // ANY MotorCommand so the bridge's per-channel ch_buf_ isn't being
      // thrashed while the arm hw_interface waits for its own DISABLE.
      return;
    }
    if (!g.has_state && g.state != GState::INIT && g.state != GState::ENABLING) {
      return;
    }
    if (g.state == GState::INIT) {
      // Keep the original arm-before-gripper startup ordering. W3 only
      // reports online feedback after the disabled gripper receives traffic,
      // so INIT/ENABLING must be allowed to send zero-impedance commands first.
      if (g.arm_armed_count != 7) {
        // Don't even publish a state-tracking heartbeat; the gripper motor
        // is still disabled, no watchdog to satisfy yet.
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
          "%s gripper: waiting for arm on ch%u (%d/7 enabled)...",
          g.side.c_str(), static_cast<unsigned>(g.channel),
          g.arm_armed_count);
        return;
      }
      // Arm is up. Send the per-motor FC enable a few times (CAN is lossy)
      // and let the DM motor settle into MIT mode before running calibration.
      enter_enabling(g);
    }

    g.phase_ticks++;

    double pos_cmd = g.q_motor;
    double vel_ff  = 0.0;
    double kp      = 0.0;
    double kd      = 0.0;
    double tau_ff  = 0.0;

    switch (g.state) {
      case GState::INIT:
      case GState::FAILED:
        pos_cmd = g.q_motor; vel_ff = 0.0; kp = 0.0; kd = 0.0; tau_ff = 0.0;
        break;

      case GState::ENABLING: {
        // Mirror dm_hardware_interface's enable strategy: hammer FC at 50 Hz
        // (configurable) AND publish a zero-impedance MotorCommand every tick
        // (kp=kd=0, vel=0, tau=0) to refresh the W3 command heartbeat, then
        // wait for the motor to report err == DM_ERR_ENABLED for several
        // consecutive frames before advancing.
        static constexpr uint8_t DM_ERR_ENABLED = 1;
        if (g.phase_ticks == 1 ||
            (g.phase_ticks % g.enable_pub_period_ticks) == 0)
        {
          publish_motor_enable(g, /*enable=*/true);
        }
        if (g.has_state && g.err == DM_ERR_ENABLED) {
          if (++g.enable_confirm_count >= g.enable_confirm_frames) {
            RCLCPP_INFO(get_logger(),
              "%s gripper: motor reports err=1 (ENABLED) for %d frames; "
              "starting calibration", g.side.c_str(), g.enable_confirm_count);
            start_calibration(g);
            return;
          }
        } else {
          g.enable_confirm_count = 0;
        }
        if (g.phase_ticks > g.enable_timeout_ticks) {
          RCLCPP_ERROR(get_logger(),
            "%s gripper: ENABLE timed out after %.1fs (last err=%u, |tau|=%.2f). "
            "Check that motor is wired on ch%u id7 and not in fault. -> FAILED",
            g.side.c_str(), g.phase_ticks * dt_,
            static_cast<unsigned>(g.err), std::abs(g.torque),
            static_cast<unsigned>(g.channel));
          g.state = GState::FAILED;
          break;
        }
        // Zero MotorCommand this tick (kp=kd=0 → motor sees no setpoint).
        pos_cmd = g.q_motor; vel_ff = 0.0; kp = 0.0; kd = 0.0; tau_ff = 0.0;
        break;
      }

      case GState::CALIBRATING_OPEN:
      case GState::CALIBRATING_CLOSE:
        run_calibration(g, pos_cmd, vel_ff, kp, kd, tau_ff);
        break;

      case GState::MOVING_OPEN:
      case GState::MOVING_CLOSE:
        run_motion(g, pos_cmd, vel_ff, kp, kd, tau_ff);
        break;

      case GState::HOLD:
        pos_cmd = g.q_held;
        vel_ff  = 0.0;
        kp = g.hold_kp;
        kd = g.hold_kd;
        tau_ff = 0.0;
        break;

      case GState::FORCE_HOLD:
        // Constant push past contact with bounded torque: target overshoots
        // the contact pose by overshoot; kp is sized so kp*overshoot = thr.
        pos_cmd = g.q_held + g.close_motor_direction * g.force_hold_overshoot;
        vel_ff  = 0.0;
        kp = (g.force_hold_overshoot > 1e-4)
               ? (g.cmd_force_thr / g.force_hold_overshoot)
               : g.hold_kp;
        kp = std::min(kp, g.hold_kp * 3.0);  // sanity cap
        kd = g.hold_kd;
        tau_ff = 0.0;
        break;

      case GState::TELEOP_PASSIVE:
        pos_cmd = g.q_motor;
        vel_ff  = 0.0;
        kp = 0.0;
        kd = g.teleop_passive_kd;
        // Match the arm gravity controller's friction convention: use
        // measured velocity, apply Coulomb+viscous outside deadband.
        tau_ff = friction_ff(g, g.velocity_filtered);
        break;

      case GState::TELEOP_TRACK:
        run_teleop_track(g, pos_cmd, vel_ff, kp, kd, tau_ff);
        break;
    }

    publish_cmd(g, pub, pos_cmd, vel_ff, kp, kd, tau_ff);
    publish_teleop_state(g);
  }

  // --------------------------------------------------------------------------
  // Calibration branch -- ramp toward direction at cal_speed; watch torque
  // for stall, then commit boundary and advance phase.
  // --------------------------------------------------------------------------
  void run_calibration(Gripper & g,
                       double & pos_cmd, double & vel_ff,
                       double & kp, double & kd, double & tau_ff)
  {
    // Integrate target at constant velocity. kp / kd low so the motor PD can
    // be back-driven if we already are at the hard stop.
    g.q_target += g.cmd_speed * dt_;
    pos_cmd = g.q_target;
    vel_ff  = g.cmd_speed;
    kp = g.cal_kp;
    kd = g.cal_kd;
    tau_ff = friction_ff(g, g.cmd_speed);

    const bool torque_loaded = std::abs(g.torque) > g.cal_stall_torque;
    const bool nearly_still  = std::abs(g.velocity) < g.cal_stall_vel_thr;
    // Require both: high reaction torque AND velocity near zero, so we don't
    // confuse "accelerating from rest" with "stalled against stop".
    if (torque_loaded && nearly_still && g.phase_ticks > 5) {
      g.stall_counter++;
    } else {
      g.stall_counter = 0;
    }

    if (g.stall_counter >= g.cal_stall_ticks) {
      if (g.state == GState::CALIBRATING_OPEN) {
        g.q_open_motor = g.q_motor;
        g.stall_counter = 0;
        g.phase_ticks = 0;
        g.cmd_speed = g.close_motor_direction * std::abs(g.cal_speed);
        g.q_target = g.q_motor;
        g.state = GState::CALIBRATING_CLOSE;
        RCLCPP_INFO(get_logger(),
          "%s gripper: OPEN boundary @ q_motor=%.3frad (|tau|=%.2fNm); "
          "now seeking CLOSE",
          g.side.c_str(), g.q_open_motor, std::abs(g.torque));
      } else {
        g.q_close_motor = g.q_motor;
        g.stroke_motor = std::abs(g.q_close_motor - g.q_open_motor);
        g.calibrated = true;
        // After calibration, retreat to OPEN to release any object.
        g.q_held = g.q_open_motor;
        g.state = GState::HOLD;  // settle there
        g.phase_ticks = 0;
        RCLCPP_INFO(get_logger(),
          "%s gripper: CLOSE boundary @ q_motor=%.3frad (|tau|=%.2fNm); "
          "stroke=%.3frad (~%.2f motor turns). Calibration DONE -> HOLD at OPEN.",
          g.side.c_str(), g.q_close_motor, std::abs(g.torque),
          g.stroke_motor, g.stroke_motor / (2.0 * M_PI));
      }
      return;
    }

    if (g.phase_ticks > g.cal_phase_timeout_ticks) {
      RCLCPP_ERROR(get_logger(),
        "%s gripper: calibration phase '%s' timed out after %.1fs without "
        "detecting stall (|tau| capped at ~%.2fNm). Check stall_torque or "
        "mechanical link. Latching to FAILED.",
        g.side.c_str(), state_str(g.state),
        g.phase_ticks * dt_, std::abs(g.torque));
      g.state = GState::FAILED;
    }
  }

  // --------------------------------------------------------------------------
  // Motion branch -- user-commanded open/close with limit + force triggers.
  // --------------------------------------------------------------------------
  void run_motion(Gripper & g,
                  double & pos_cmd, double & vel_ff,
                  double & kp, double & kd, double & tau_ff)
  {
    // Integrate target. Clamp against calibrated stroke +/- small margin to
    // avoid driving past the hard stop.
    g.q_target += g.cmd_speed * dt_;
    const double q_min = std::min(g.q_open_motor, g.q_close_motor);
    const double q_max = std::max(g.q_open_motor, g.q_close_motor);
    g.q_target = std::clamp(g.q_target, q_min, q_max);

    pos_cmd = g.q_target;
    vel_ff  = g.cmd_speed;
    kp = g.move_kp;
    kd = g.move_kd;
    tau_ff = friction_ff(g, g.cmd_speed);

    if (g.state == GState::MOVING_OPEN) {
      // OPEN has the highest priority -- run all the way to q_open_motor
      // regardless of torque. The whole point of "open" is to release
      // whatever the gripper is currently grasping; force-gating it would
      // make a stuck close-position freeze the gripper. Position is the
      // only stop condition here.
      const bool past_open =
        (g.cmd_speed > 0.0 && g.q_motor >= g.q_open_motor) ||
        (g.cmd_speed < 0.0 && g.q_motor <= g.q_open_motor);
      if (past_open) {
        g.q_held = g.q_open_motor;
        g.state = GState::HOLD;
        RCLCPP_INFO(get_logger(),
          "%s gripper: reached OPEN -> HOLD @%.3frad", g.side.c_str(), g.q_held);
      }
    } else {  // MOVING_CLOSE
      // Force gate: |tau| > threshold (after grace) -> FORCE_HOLD at contact.
      if (g.phase_ticks > g.force_grace_ticks &&
          std::abs(g.torque) > g.cmd_force_thr)
      {
        g.q_held = g.q_motor;
        g.state = GState::FORCE_HOLD;
        RCLCPP_INFO(get_logger(),
          "%s gripper: |tau|=%.2fNm > %.2fNm -> FORCE_HOLD @%.3frad",
          g.side.c_str(), std::abs(g.torque), g.cmd_force_thr, g.q_held);
        return;
      }
      // Reached calibrated close position without an object?
      const bool past_close =
        (g.cmd_speed > 0.0 && g.q_motor >= g.q_close_motor) ||
        (g.cmd_speed < 0.0 && g.q_motor <= g.q_close_motor);
      if (past_close) {
        g.q_held = g.q_close_motor;
        g.state = GState::HOLD;
        RCLCPP_INFO(get_logger(),
          "%s gripper: reached CLOSE limit (no object) -> HOLD @%.3frad",
          g.side.c_str(), g.q_held);
      }
    }
  }

  void run_teleop_track(Gripper & g,
                        double & pos_cmd, double & vel_ff,
                        double & kp, double & kd, double & tau_ff)
  {
    const double desired = ratio_to_motor(g, g.teleop_target_ratio);
    const double max_step = std::max(1e-5, g.teleop_track_max_step);
    const double delta = std::clamp(desired - g.q_teleop_target, -max_step, max_step);
    g.q_teleop_target += delta;

    pos_cmd = g.q_teleop_target;
    vel_ff = delta / std::max(1e-6, dt_);
    kp = g.teleop_track_kp;
    kd = g.teleop_track_kd;
    tau_ff = friction_ff(g, vel_ff);
  }

  // --------------------------------------------------------------------------
  // Publish per-motor ENABLE / DISABLE to the W3 bridge so the DM motor
  // enters / leaves MIT mode. CAN is lossy so the ENABLING state machine
  // calls this a few times spaced out by enable_pub_interval.
  // --------------------------------------------------------------------------
  void publish_motor_enable(
    const Gripper & g,
    bool enable)
  {
    if (!cmd_pub_) {
      return;
    }
    w3_robot_bridge::msg::MotorCommandArray msg;
    msg.header.stamp = now();
    msg.commands.resize(1);
    auto & m = msg.commands[0];
    m.channel = g.channel;
    m.motor_index = g.slot;
    m.mode = enable ?
      w3_robot_bridge::msg::MotorCommand::MODE_ENABLE :
      w3_robot_bridge::msg::MotorCommand::MODE_DISABLE;
    cmd_pub_->publish(msg);
  }

  // --------------------------------------------------------------------------
  // Friction feed-forward at desired velocity (gated by deadband).
  // --------------------------------------------------------------------------
  static double friction_ff(const Gripper & g, double v_des)
  {
    if (!g.fric_enabled) {
      return 0.0;
    }
    if (std::abs(v_des) < g.fric_deadband) {
      return 0.0;
    }
    const double coulomb = (v_des > 0.0) ? g.fric_coulomb_pos : -g.fric_coulomb_neg;
    return g.fric_gain * (coulomb + g.fric_viscous * v_des);
  }

  void publish_teleop_state(const Gripper & g)
  {
    const auto pub = (g.side == "left") ? left_teleop_state_pub_ : right_teleop_state_pub_;
    if (!pub || pub->get_subscription_count() == 0) {
      return;
    }
    std_msgs::msg::Float32MultiArray msg;
    msg.data.resize(6);
    msg.data[0] = static_cast<float>(gripper_ratio(g));
    msg.data[1] = static_cast<float>(g.q_motor);
    msg.data[2] = static_cast<float>(g.velocity);
    msg.data[3] = static_cast<float>(g.torque);
    msg.data[4] = g.calibrated ? 1.0f : 0.0f;
    msg.data[5] = static_cast<float>(static_cast<int>(g.state));
    pub->publish(msg);
  }

  // --------------------------------------------------------------------------
  // Publish MotorCommandArray containing exactly the one slot-7 command.
  // --------------------------------------------------------------------------
  void publish_cmd(const Gripper & g,
                   const rclcpp::Publisher<w3_robot_bridge::msg::MotorCommandArray>::SharedPtr & pub,
                   double pos, double vel, double kp, double kd, double tau)
  {
    if (!pub) {
      return;
    }
    w3_robot_bridge::msg::MotorCommandArray msg;
    msg.header.stamp = now();
    msg.commands.resize(1);
    auto & m = msg.commands[0];
    m.channel = g.channel;
    m.motor_index = g.slot;
    m.mode = w3_robot_bridge::msg::MotorCommand::MODE_RUN;
    m.position  = static_cast<float>(std::clamp(pos, -g.pos_max * 10.0, g.pos_max * 10.0));
    // Note: dm_motor_bridge will clamp position into [-pos_max, +pos_max] for
    // the MIT packet anyway; we send the raw multi-turn target which the
    // motor PD will treat as "very far away" once outside the window. The
    // motor PD effectively becomes a soft velocity loop driven by kp+kd.
    m.velocity  = static_cast<float>(std::clamp(vel, -g.vel_max, g.vel_max));
    m.kp        = static_cast<float>(std::max(0.0, kp));
    m.kd        = static_cast<float>(std::max(0.0, kd));
    m.torque    = static_cast<float>(std::clamp(tau, -g.tor_max, g.tor_max));
    pub->publish(msg);
  }

  // --------------------------------------------------------------------------
  // 1 Hz textual diagnostic.
  // --------------------------------------------------------------------------
  void publish_status(const Gripper & g,
                      const rclcpp::Publisher<std_msgs::msg::String>::SharedPtr & pub)
  {
    if (!g.enabled || !pub) {
      return;
    }
    if (pub->get_subscription_count() == 0) {
      return;
    }
    std::lock_guard<std::mutex> lock(mu_);
    char buf[256];
    std::snprintf(buf, sizeof(buf),
      "state:%s calibrated:%d ratio:%.3f q_motor:%.3f raw:%.3f v:%.2f vf:%.2f tau:%.2f "
      "open:%.3f close:%.3f stroke:%.3f err:%u",
      state_str(g.state), g.calibrated ? 1 : 0,
      gripper_ratio(g), g.q_motor, g.raw_pos, g.velocity, g.velocity_filtered, g.torque,
      g.q_open_motor, g.q_close_motor, g.stroke_motor,
      static_cast<unsigned>(g.err));
    std_msgs::msg::String m;
    m.data = buf;
    pub->publish(m);
  }

  // --------------------------------------------------------------------------
  Gripper left_;
  Gripper right_;
  double dt_ = 1.0 / 500.0;
  std::mutex mu_;

  rclcpp::Subscription<w3_robot_bridge::msg::MotorStateArray>::SharedPtr state_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr left_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr right_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr left_teleop_target_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr right_teleop_target_sub_;
  rclcpp::Publisher<w3_robot_bridge::msg::MotorCommandArray>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr left_status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr right_status_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr left_teleop_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr right_teleop_state_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;

public:
  // Send FD (disable) to both grippers; one shot.
  void disable_motors()
  {
    if (left_.enabled && cmd_pub_) {
      publish_motor_enable(left_, false);
    }
    if (right_.enabled && cmd_pub_) {
      publish_motor_enable(right_, false);
    }
  }

  // Synchronous shutdown disable. User-requested ordering:
  //   1) WAIT for the arm hw_interface to finish its own DISABLE -- detected
  //      via W3 state (slots 0..6 enabled -> false). During this wait we
  //      STOP publishing any gripper MotorCommand so the bridge's ch_buf_
  //      isn't being thrashed between arm & gripper writers (same conflict
  //      that caused arm enable to flake during startup). The gripper motor
  //      will hit its ~100 ms watchdog and self-disable -- that's fine, we
  //      were going to FD it anyway.
  //   2) THEN publish FD at 5 Hz to slot 7 until both gripper motors report
  //      err == 0, or `gripper_fd_timeout_s` elapses.
  // Caller is responsible for keeping the executor spinning.
  void shutdown_disable_sequence(rclcpp::executors::SingleThreadedExecutor & exec,
                                 double arm_wait_timeout_s   = 4.0,
                                 double gripper_fd_timeout_s = 3.0)
  {
    using namespace std::chrono;
    static constexpr uint8_t DM_ERR_DISABLED = 0;

    // ---------- Phase 1: pause gripper publishing & wait for arm ----------
    {
      // Set state to FAILED so step() takes the early-out tau=kp=kd=0
      // branch... but ALSO we want it to skip publishing entirely so the
      // arm's FD wait isn't fighting us on ch_buf_[ch]. Easiest way: set a
      // shared flag and have step() bail at the top.
      std::lock_guard<std::mutex> lock(mu_);
      shutdown_in_progress_.store(true);
      if (left_.enabled)  left_.state  = GState::FAILED;
      if (right_.enabled) right_.state = GState::FAILED;
    }

    RCLCPP_INFO(get_logger(),
      "Shutdown phase 1: paused gripper cmd publishing; waiting for arm "
      "DISABLE on can0 & can1 (timeout %.1fs)...", arm_wait_timeout_s);

    const auto t0 = steady_clock::now();
    while (rclcpp::ok()) {
      const auto now = steady_clock::now();
      const auto elapsed = duration<double>(now - t0).count();
      if (elapsed > arm_wait_timeout_s) {
        RCLCPP_WARN(get_logger(),
          "Shutdown phase 1 timed out after %.1fs "
          "(left arm armed=%d/7, right arm armed=%d/7); "
          "proceeding to gripper FD anyway.",
          elapsed, left_.arm_armed_count, right_.arm_armed_count);
        break;
      }
      exec.spin_some(milliseconds(10));
      const bool left_arm_off  = !left_.enabled  || left_.arm_armed_count  == 0;
      const bool right_arm_off = !right_.enabled || right_.arm_armed_count == 0;
      if (left_arm_off && right_arm_off) {
        RCLCPP_INFO(get_logger(),
          "Shutdown phase 1: arm DISABLED (took %.2fs); proceeding to gripper FD.",
          elapsed);
        break;
      }
      std::this_thread::sleep_for(milliseconds(20));
    }

    // ---------- Phase 2: gripper FD until err==0 ----------
    RCLCPP_INFO(get_logger(),
      "Shutdown phase 2: 5Hz DISABLE on W3 bridge until both "
      "gripper motors report err=0 (timeout %.1fs)...", gripper_fd_timeout_s);

    const auto t1 = steady_clock::now();
    const auto fd_period = milliseconds(200);  // 5 Hz
    auto last_fd = t1 - seconds(1);

    while (rclcpp::ok()) {
      const auto now = steady_clock::now();
      const auto elapsed = duration<double>(now - t1).count();
      if (elapsed > gripper_fd_timeout_s) {
        RCLCPP_WARN(get_logger(),
          "Shutdown phase 2 timed out after %.1fs (left.err=%u, right.err=%u)",
          elapsed,
          static_cast<unsigned>(left_.err),
          static_cast<unsigned>(right_.err));
        return;
      }

      if (now - last_fd >= fd_period) {
        disable_motors();
        last_fd = now;
      }
      exec.spin_some(milliseconds(10));

      const bool left_off  = !left_.enabled  || left_.err == DM_ERR_DISABLED;
      const bool right_off = !right_.enabled || right_.err == DM_ERR_DISABLED;
      if (left_off && right_off) {
        RCLCPP_INFO(get_logger(),
          "Shutdown phase 2: both grippers DISABLED "
          "(left.err=%u, right.err=%u, took %.2fs)",
          static_cast<unsigned>(left_.err),
          static_cast<unsigned>(right_.err),
          elapsed);
        return;
      }

      std::this_thread::sleep_for(milliseconds(5));
    }
  }

private:
  // Set true at start of shutdown_disable_sequence; checked by step() to skip
  // ALL gripper command publishing so arm hw_interface's deactivate phase
  // isn't disturbed.
  std::atomic<bool> shutdown_in_progress_{false};
};

}  // namespace eiriarm_controllers

namespace {
  // Set by our SIGINT/SIGTERM handler. Polled by main loop to leave the
  // executor cleanly *before* rclcpp's context tears down its publishers.
  std::atomic<bool> g_quit_requested{false};
}

int main(int argc, char ** argv)
{
  // Disable rclcpp's default SIGINT handler. Its on_shutdown callbacks fire
  // *after* the context begins tearing down, which has been observed to
  // invalidate publishers mid-callback ("publisher's context is invalid")
  // -- the FD frame for the gripper motors then never reaches the bridge.
  // Instead we install our own signal handler, leave spin() cleanly while
  // the context is still healthy, run a synchronous DISABLE wait, *then*
  // call rclcpp::shutdown().
  rclcpp::InitOptions init_opts;
  rclcpp::init(argc, argv, init_opts, rclcpp::SignalHandlerOptions::None);

  auto handler = [](int /*sig*/) { g_quit_requested.store(true); };
  std::signal(SIGINT,  handler);
  std::signal(SIGTERM, handler);

  auto node = std::make_shared<eiriarm_controllers::GripperController>();
  RCLCPP_INFO(node->get_logger(), "Gripper controller started (real hardware mode)");

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);

  while (rclcpp::ok() && !g_quit_requested.load()) {
    exec.spin_some(std::chrono::milliseconds(10));
  }

  // Synchronous disable: keep spinning the executor (so the 500 Hz control
  // timer fires zero-cmd MotorCommand frames + on_state updates `err`) while
  // we hammer FD at 5 Hz, until both grippers report err == 0 or 3 s elapse.
  RCLCPP_INFO(node->get_logger(),
              "Quit requested -- entering synchronous gripper DISABLE...");
  node->shutdown_disable_sequence(exec, /*timeout_s=*/3.0);

  // One final pause to let the last FD's CAN frame egress before the context
  // is torn down.
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  rclcpp::shutdown();
  return 0;
}
