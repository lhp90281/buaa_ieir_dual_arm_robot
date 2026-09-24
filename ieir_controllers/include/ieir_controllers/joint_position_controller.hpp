#ifndef IEIR_CONTROLLERS__JOINT_POSITION_CONTROLLER_HPP_
#define IEIR_CONTROLLERS__JOINT_POSITION_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace ieir_controllers
{

/**
 * @brief Joint-space position controller for DM motors in MIT mode.
 *
 * The DM motor itself runs a PD inner loop:
 *     tau_motor = kp*(pos_cmd - pos_actual) + kd*(vel_cmd - vel_actual)
 *               + torque_ff
 * so this controller does not have to compute torques itself. It claims
 *   <joint>/position, <joint>/velocity, <joint>/stiffness, <joint>/damping
 * command interfaces and forwards a setpoint coming from a
 * trajectory_msgs/JointTrajectory topic, while keeping kp/kd at static
 * per-joint values read from the parameter server.
 *
 * Designed to run in parallel with `gravity_compensation_controller`,
 * which separately claims the `effort` (torque_ff) interface. The motor
 * sees the sum:
 *     tau_motor = kp*(q_des - q) + kd*(qdot_des - qdot)
 *               + tau_gravity + tau_friction
 * i.e. gravity-compensated joint impedance.
 *
 * Parameters (yaml):
 *   joints:    [j0, j1, ...]                   # ordered list of joints
 *   kp_gains:  [kp0, kp1, ...]                 # one per joint, same order
 *   kd_gains:  [kd0, kd1, ...]                 # one per joint, same order
 *   command_topic: "/joint_position_command"   # JointTrajectory topic
 *
 * Trajectory semantics:
 *   - header.stamp == 0  -> start "now" (this update() iteration)
 *   - header.stamp != 0  -> start at that absolute time
 *   - points[i].time_from_start is relative to that start time
 *   - within a segment we linearly interpolate position
 *     (and use the segment slope as the velocity feed-forward)
 *   - after the last point we hold the last point's position with vel=0
 *   - a partial joint set in trajectory.joint_names is OK; missing joints
 *     hold their previous setpoint
 *
 * On activation the controller initializes the held setpoint from the
 * current measured joint positions, so the very first publish is bumpless
 * even if kp/kd are non-zero (the motor PD sees q_des = q).
 */
class JointPositionController : public controller_interface::ControllerInterface
{
public:
  JointPositionController();

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
  // ---- helpers ----
  // Take the latest trajectory from the RT buffer (if any) and install it
  // as the active trajectory, building the joint index map and resolving
  // the reference time. Called at the top of update().
  void try_pickup_new_trajectory(const rclcpp::Time & now);

  // Compute the desired (position, velocity) for joint local index
  // `local_idx` (into joint_names_) at absolute time `now`. Falls back to
  // the held setpoint with vel=0 when no trajectory is active or the
  // joint is not in the trajectory.
  void sample_setpoint(size_t local_idx, const rclcpp::Time & now,
                       double & pos_out, double & vel_out) const;

  // ---- parameters ----
  std::vector<std::string> joint_names_;
  std::vector<double> kp_gains_;        // size == joint_names_.size()
  std::vector<double> kd_gains_;        // size == joint_names_.size()
  std::string command_topic_;

  // ---- ROS plumbing ----
  // RT-safe handoff: subscriber callback writes here, update() reads.
  realtime_tools::RealtimeBuffer<std::shared_ptr<trajectory_msgs::msg::JointTrajectory>>
    traj_buffer_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_sub_;

  // ---- active trajectory state (touched only inside update()) ----
  std::shared_ptr<trajectory_msgs::msg::JointTrajectory> active_traj_;
  rclcpp::Time active_traj_start_;
  // For each local joint i, index of that joint in active_traj_->joint_names,
  // or SIZE_MAX if the trajectory does not constrain that joint.
  std::vector<size_t> traj_joint_idx_;

  // ---- held setpoint (per local joint, URDF frame) ----
  // Last position we commanded. Used when no trajectory is active or for
  // joints absent from the trajectory. Initialized from measured state on
  // on_activate() so the first published cmd_pos == current pos -> bumpless.
  std::vector<double> hold_pos_;

  // Snapshot of hold_pos_ taken at the moment a new trajectory was picked
  // up. Used ONLY as q_prev for the pre-roll segment (before pts[0]) so
  // that the ramp is linear from "where we were when the traj arrived" to
  // pts[0]. We cannot read live hold_pos_ for this because update()
  // overwrites it every tick with the latest setpoint -- which collapses
  // pre-roll into an exponential decay instead of a linear ramp.
  std::vector<double> pre_roll_start_pos_;

  // ---- cached interface indices (filled in on_activate) ----
  // command_interfaces_ is laid out [j0/pos, j0/vel, j0/stiff, j0/damp,
  //                                  j1/pos, j1/vel, j1/stiff, j1/damp, ...]
  // matching command_interface_configuration(). State is similarly
  // [j0/pos, j0/vel, j1/pos, j1/vel, ...]. We resolve names once at
  // activation rather than scanning every update().
  static constexpr size_t kCmdStride = 4;
  static constexpr size_t kStateStride = 2;

  static constexpr size_t SIZE_MAX_LOCAL = static_cast<size_t>(-1);
};

}  // namespace ieir_controllers

#endif  // IEIR_CONTROLLERS__JOINT_POSITION_CONTROLLER_HPP_
