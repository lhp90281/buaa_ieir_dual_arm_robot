#include "ieir_controllers/joint_position_controller.hpp"

#include <algorithm>
#include <limits>

#include "controller_interface/helpers.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace ieir_controllers
{

namespace
{
// builtin_interfaces::Duration -> seconds (as double).
inline double duration_to_sec(const builtin_interfaces::msg::Duration & d)
{
  return static_cast<double>(d.sec) + static_cast<double>(d.nanosec) * 1e-9;
}
}  // namespace

JointPositionController::JointPositionController() = default;

controller_interface::InterfaceConfiguration
JointPositionController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  // IMPORTANT: keep this layout in sync with kCmdStride in the header.
  // For joint i, the command interface indices are:
  //   4*i + 0: position
  //   4*i + 1: velocity
  //   4*i + 2: stiffness  (== motor kp)
  //   4*i + 3: damping    (== motor kd)
  for (const auto & name : joint_names_) {
    cfg.names.push_back(name + "/" + hardware_interface::HW_IF_POSITION);
    cfg.names.push_back(name + "/" + hardware_interface::HW_IF_VELOCITY);
    cfg.names.push_back(name + "/stiffness");
    cfg.names.push_back(name + "/damping");
  }
  return cfg;
}

controller_interface::InterfaceConfiguration
JointPositionController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  // For joint i, the state interface indices are:
  //   2*i + 0: position
  //   2*i + 1: velocity
  for (const auto & name : joint_names_) {
    cfg.names.push_back(name + "/" + hardware_interface::HW_IF_POSITION);
    cfg.names.push_back(name + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  return cfg;
}

controller_interface::CallbackReturn JointPositionController::on_init()
{
  try {
    auto_declare<std::vector<std::string>>("joints", {});
    auto_declare<std::vector<double>>("kp_gains", {});
    auto_declare<std::vector<double>>("kd_gains", {});
    auto_declare<std::string>("command_topic", "/joint_position_command");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "JointPositionController on_init failed: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointPositionController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto node = get_node();

  joint_names_ = node->get_parameter("joints").as_string_array();
  if (joint_names_.empty()) {
    RCLCPP_ERROR(node->get_logger(), "'joints' parameter is empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  kp_gains_ = node->get_parameter("kp_gains").as_double_array();
  kd_gains_ = node->get_parameter("kd_gains").as_double_array();
  if (kp_gains_.size() != joint_names_.size() ||
      kd_gains_.size() != joint_names_.size())
  {
    RCLCPP_ERROR(node->get_logger(),
                 "kp_gains (%zu) and kd_gains (%zu) must both have size == joints (%zu)",
                 kp_gains_.size(), kd_gains_.size(), joint_names_.size());
    return controller_interface::CallbackReturn::ERROR;
  }
  // Reject negatives -- DM expects non-negative kp/kd.
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    if (kp_gains_[i] < 0.0 || kd_gains_[i] < 0.0) {
      RCLCPP_ERROR(node->get_logger(),
                   "Joint '%s': negative gain (kp=%.3f, kd=%.3f) is not allowed",
                   joint_names_[i].c_str(), kp_gains_[i], kd_gains_[i]);
      return controller_interface::CallbackReturn::ERROR;
    }
  }

  command_topic_ = node->get_parameter("command_topic").as_string();

  // Subscribe in non-RT thread; hand off to update() via realtime buffer.
  // The buffer holds at most one trajectory; new ones replace the previous
  // (queue depth 1 effectively), which matches the "latest goal wins"
  // semantics typical of impedance control.
  traj_sub_ = node->create_subscription<trajectory_msgs::msg::JointTrajectory>(
    command_topic_, rclcpp::SystemDefaultsQoS(),
    [this](const trajectory_msgs::msg::JointTrajectory::SharedPtr msg) {
      // Defensive copy so the subscriber-thread shared_ptr cannot be
      // mutated under the RT thread's feet.
      auto copy = std::make_shared<trajectory_msgs::msg::JointTrajectory>(*msg);
      traj_buffer_.writeFromNonRT(copy);
    });

  RCLCPP_INFO(node->get_logger(),
              "JointPositionController configured with %zu joint(s); "
              "subscribing to '%s' (trajectory_msgs/JointTrajectory)",
              joint_names_.size(), command_topic_.c_str());
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    RCLCPP_INFO(node->get_logger(),
                "  %-14s  kp=%6.2f  kd=%6.3f",
                joint_names_[i].c_str(), kp_gains_[i], kd_gains_[i]);
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointPositionController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto node = get_node();
  const size_t n = joint_names_.size();

  // Sanity: ros2_control must have laid out interfaces in the same order
  // we requested in command_interface_configuration() / state_interface_configuration().
  if (command_interfaces_.size() != n * kCmdStride) {
    RCLCPP_ERROR(node->get_logger(),
                 "Expected %zu command interfaces, got %zu",
                 n * kCmdStride, command_interfaces_.size());
    return controller_interface::CallbackReturn::ERROR;
  }
  if (state_interfaces_.size() != n * kStateStride) {
    RCLCPP_ERROR(node->get_logger(),
                 "Expected %zu state interfaces, got %zu",
                 n * kStateStride, state_interfaces_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  // Bumpless start: hold setpoint = current measured position. The first
  // update() will publish (cmd_pos, cmd_vel) = (current pos, 0) with the
  // yaml kp/kd, so the motor PD computes ~zero correction and the robot
  // does not jump even though kp just went from 0 to a non-zero value.
  hold_pos_.assign(n, 0.0);
  for (size_t i = 0; i < n; ++i) {
    hold_pos_[i] = state_interfaces_[i * kStateStride + 0].get_value();
  }
  // No trajectory has been picked up yet, so the pre-roll snapshot is
  // simply the activation-time pose. It will be overwritten the moment
  // try_pickup_new_trajectory() accepts a trajectory.
  pre_roll_start_pos_ = hold_pos_;

  // Drop any trajectory that arrived while we were inactive -- those
  // setpoints are stale and starting from them would defeat the bumpless
  // semantics above.
  traj_buffer_.reset();
  active_traj_.reset();
  traj_joint_idx_.clear();

  RCLCPP_INFO(node->get_logger(),
              "JointPositionController activated; holding current position. "
              "Send a trajectory_msgs/JointTrajectory on '%s' to move.",
              command_topic_.c_str());
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointPositionController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Hand control back gracefully: zero the gains and the velocity command.
  // With kp == kd == 0 the dm_hardware_interface's bumpless mirror kicks
  // in and tracks the live motor position, so even if no other controller
  // takes over the motor cannot snap toward a stale target.
  // (cmd_pos is overwritten by that mirror too, but we set it to current
  // measured position here to be explicit and self-documenting.)
  const size_t n = joint_names_.size();
  for (size_t i = 0; i < n; ++i) {
    const double q = state_interfaces_[i * kStateStride + 0].get_value();
    command_interfaces_[i * kCmdStride + 0].set_value(q);     // position
    command_interfaces_[i * kCmdStride + 1].set_value(0.0);   // velocity
    command_interfaces_[i * kCmdStride + 2].set_value(0.0);   // stiffness (kp)
    command_interfaces_[i * kCmdStride + 3].set_value(0.0);   // damping   (kd)
  }
  active_traj_.reset();
  traj_joint_idx_.clear();
  return controller_interface::CallbackReturn::SUCCESS;
}

void JointPositionController::try_pickup_new_trajectory(const rclcpp::Time & now)
{
  auto * latest = traj_buffer_.readFromRT();
  if (latest == nullptr || !*latest) {
    return;
  }
  // Ownership transfer: take the pointer and clear the slot so we do not
  // re-install the same trajectory on the next update().
  std::shared_ptr<trajectory_msgs::msg::JointTrajectory> new_traj = *latest;
  traj_buffer_.writeFromNonRT(nullptr);

  if (new_traj->points.empty()) {
    RCLCPP_WARN(get_node()->get_logger(),
                "Received JointTrajectory with no points; ignoring");
    return;
  }

  // Build joint index mapping: traj_joint_idx_[i] is the index within
  // new_traj->joint_names whose name matches joint_names_[i], or
  // SIZE_MAX_LOCAL if the joint is not present in the trajectory.
  // Joints absent from the trajectory hold their previous setpoint, which
  // matches ros2_controllers' joint_trajectory_controller convention.
  traj_joint_idx_.assign(joint_names_.size(), SIZE_MAX_LOCAL);
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    auto it = std::find(new_traj->joint_names.begin(),
                        new_traj->joint_names.end(),
                        joint_names_[i]);
    if (it != new_traj->joint_names.end()) {
      traj_joint_idx_[i] =
        static_cast<size_t>(std::distance(new_traj->joint_names.begin(), it));
    }
  }

  // Resolve trajectory start time:
  //   stamp == 0  -> "start now" (ros2 convention)
  //   stamp != 0  -> absolute start time
  const rclcpp::Time stamp(new_traj->header.stamp);
  active_traj_start_ = (stamp.nanoseconds() == 0) ? now : stamp;
  active_traj_ = new_traj;

  // Freeze the pre-roll start pose at the moment of pickup. sample_setpoint
  // will linearly interpolate from this snapshot to pts[0] over
  // [0, pts[0].time_from_start]. Without this snapshot, update() would
  // overwrite hold_pos_ each tick with the freshly-interpolated value,
  // turning the linear ramp into an exponential decay.
  pre_roll_start_pos_ = hold_pos_;

  RCLCPP_INFO(get_node()->get_logger(),
              "Picked up trajectory: %zu point(s), %zu joint(s) constrained, "
              "duration %.3fs",
              active_traj_->points.size(),
              std::count_if(traj_joint_idx_.begin(), traj_joint_idx_.end(),
                            [](size_t v){ return v != SIZE_MAX_LOCAL; }),
              duration_to_sec(active_traj_->points.back().time_from_start));
}

void JointPositionController::sample_setpoint(
  size_t i, const rclcpp::Time & now,
  double & pos_out, double & vel_out) const
{
  // Default: hold the last commanded position with zero velocity.
  pos_out = hold_pos_[i];
  vel_out = 0.0;

  if (!active_traj_ || active_traj_->points.empty()) return;
  if (i >= traj_joint_idx_.size()) return;
  const size_t kj = traj_joint_idx_[i];
  if (kj == SIZE_MAX_LOCAL) return;  // joint not constrained by this traj

  const double t_elapsed = (now - active_traj_start_).seconds();

  const auto & pts = active_traj_->points;

  // Before the first point: stay at the start of the trajectory. We use
  // the first point's position rather than hold_pos_ so the user gets a
  // well-defined "future trajectory" behavior even if hold_pos_ drifted.
  if (t_elapsed <= 0.0) {
    if (kj < pts.front().positions.size()) {
      pos_out = pts.front().positions[kj];
    }
    vel_out = 0.0;
    return;
  }

  // After the last point: hold final position.
  const double t_end = duration_to_sec(pts.back().time_from_start);
  if (t_elapsed >= t_end) {
    if (kj < pts.back().positions.size()) {
      pos_out = pts.back().positions[kj];
    }
    vel_out = 0.0;
    return;
  }

  // Find the segment [pts[seg-1], pts[seg]] that t_elapsed falls into.
  // pts[0].time_from_start may be > 0 (gap before motion begins): treat
  // that gap as "linear interp from hold_pos_ to pts[0].positions".
  size_t seg = 0;
  for (; seg < pts.size(); ++seg) {
    if (duration_to_sec(pts[seg].time_from_start) > t_elapsed) break;
  }
  // seg is now the first point with time_from_start > t_elapsed.
  // (We already handled t_elapsed >= t_end above, so seg < pts.size() here.)

  const double t_next = duration_to_sec(pts[seg].time_from_start);
  double t_prev;
  double q_prev, q_next;
  if (seg == 0) {
    // Pre-roll segment: ramp from the position at trajectory-pickup time
    // to pts[0] over [0, t_next]. Uses the frozen snapshot, NOT live
    // hold_pos_ (which update() overwrites every tick).
    t_prev = 0.0;
    q_prev = pre_roll_start_pos_[i];
  } else {
    t_prev = duration_to_sec(pts[seg - 1].time_from_start);
    if (kj < pts[seg - 1].positions.size()) {
      q_prev = pts[seg - 1].positions[kj];
    } else {
      q_prev = hold_pos_[i];
    }
  }
  if (kj < pts[seg].positions.size()) {
    q_next = pts[seg].positions[kj];
  } else {
    q_next = q_prev;
  }

  const double dt = t_next - t_prev;
  if (dt <= 1e-9) {
    pos_out = q_next;
    vel_out = 0.0;
    return;
  }
  const double alpha = (t_elapsed - t_prev) / dt;
  pos_out = q_prev + alpha * (q_next - q_prev);
  // Velocity feed-forward = segment slope. Trajectories that supply
  // explicit per-point velocities are *not* honored here -- with linear
  // position interp, the slope is the only velocity profile that is
  // self-consistent; honoring user-supplied velocities would create a
  // pos/vel mismatch that the motor PD would fight against.
  vel_out = (q_next - q_prev) / dt;
}

controller_interface::return_type JointPositionController::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  try_pickup_new_trajectory(time);

  const size_t n = joint_names_.size();
  for (size_t i = 0; i < n; ++i) {
    double pos_des, vel_des;
    sample_setpoint(i, time, pos_des, vel_des);

    // Update the hold so that:
    //   (a) when the trajectory ends, hold_pos_ already equals the final
    //       setpoint, so the controller naturally stays put;
    //   (b) on the next on_activate transition (after deactivate), the
    //       starting position is the most recent commanded one rather
    //       than a measurement that may drift under load.
    hold_pos_[i] = pos_des;

    command_interfaces_[i * kCmdStride + 0].set_value(pos_des);
    command_interfaces_[i * kCmdStride + 1].set_value(vel_des);
    command_interfaces_[i * kCmdStride + 2].set_value(kp_gains_[i]);
    command_interfaces_[i * kCmdStride + 3].set_value(kd_gains_[i]);
  }
  return controller_interface::return_type::OK;
}

}  // namespace ieir_controllers

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  ieir_controllers::JointPositionController,
  controller_interface::ControllerInterface)
