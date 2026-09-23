#pragma once

#include "w3_robot_bridge/motor_types.hpp"

#include <array>
#include <memory>
#include <string>

namespace w3_robot_bridge { class TigertoeChannel; }

namespace w3_robot_bridge {

/// Load a per-motor YAML config file and populate a MotorProfile.
/// Returns true on success, false if the file cannot be parsed.
bool load_motor_profile(const std::string& yaml_path, MotorProfile& profile);

/// Load the main bridge configuration YAML file.
/// Populates the channels array with ChannelConfig for each CAN interface.
/// If a tigertoe channel is found, creates it in *tigertoe_out (can be nullptr).
/// Returns true on success.
bool load_bridge_config(const std::string& yaml_path,
                        std::array<ChannelConfig, kNumChannels>& channels,
                        std::unique_ptr<TigertoeChannel>* tigertoe_out = nullptr);

// ── Tigertoe (钛虎) config loading ────────────────────────────

/// Load a single tigertoe motor YAML config file.
bool load_tigertoe_motor_profile(const std::string& yaml_path,
                                 TigertoeMotorConfig& cfg);

/// Load all tigertoe motor configs from a list of YAML paths.
/// Appends results to the profiles vector.
bool load_tigertoe_configs(const std::vector<std::string>& yaml_paths,
                           std::vector<TigertoeMotorConfig>& profiles);

}  // namespace w3_robot_bridge
