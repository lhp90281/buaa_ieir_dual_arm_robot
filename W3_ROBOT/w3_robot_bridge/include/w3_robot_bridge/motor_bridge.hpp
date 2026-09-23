#pragma once

#include "w3_robot_bridge/motor_types.hpp"
#include "w3_robot_bridge/can_channel.hpp"
#include "w3_robot_bridge/tigertoe_channel.hpp"

#include <array>
#include <memory>
#include <string>
#include <vector>

namespace w3_robot_bridge {

/// Top-level bridge: manages all CAN FD channels and provides a unified
/// motor control API for the ROS node layer.
///
/// Owns all CanChannel instances. The ROS node uses this class to:
///   - Dispatch incoming ROS commands to the correct channel/motor
///   - Collect motor state for ROS state publishing
///   - Handle enable/disable/zero operations
class MotorBridge {
public:
    MotorBridge()  = default;
    ~MotorBridge();

    MotorBridge(const MotorBridge&)            = delete;
    MotorBridge& operator=(const MotorBridge&) = delete;

    /// Initialize all channels from a main config YAML file.
    /// Creates CanChannel instances only for active channels.
    bool init(const std::string& config_path);

    /// Start all active channels (launches TX/RX threads).
    bool start();

    /// Stop all channels.
    void stop();

    /// Close all channels and release resources.
    void close();

    // ── Command interface (used by ROS subscribers) ────────────
    /// Set command for a specific motor.
    void setCommand(uint8_t channel, uint8_t motor_index, const MotorCommand& cmd);

    /// Enable/disable/zero a single motor.
    void enableMotor(uint8_t channel, uint8_t motor_index);
    void disableMotor(uint8_t channel, uint8_t motor_index);
    void setZero(uint8_t channel, uint8_t motor_index);

    /// Batch operations — operate on all active motors across all channels.
    void enableAll();
    void disableAll();

    // ── State access ───────────────────────────────────────────
    MotorState getState(uint8_t channel, uint8_t motor_index) const;
    bool isOnline(uint8_t channel, uint8_t motor_index) const;
    bool isEnabled(uint8_t channel, uint8_t motor_index) const;
    const MotorProfile* getProfile(uint8_t channel, uint8_t motor_index) const;

    // ── Channel access ─────────────────────────────────────────
    const std::vector<uint8_t>& activeChannels() const { return active_channels_; }
    CanChannel* getChannel(uint8_t ch);
    const CanChannel* getChannel(uint8_t ch) const;
    uint8_t motorCount(uint8_t channel) const;

    // ── Tigertoe (钛虎) access ─────────────────────────────────
    TigertoeChannel* getTigertoeChannel() { return tigertoe_channel_.get(); }
    bool hasTigertoeChannel() const { return tigertoe_channel_ != nullptr; }

    // ── Config access ──────────────────────────────────────────
    double controlRateHz() const { return control_rate_hz_; }

private:
    std::array<std::unique_ptr<CanChannel>, kNumChannels> channels_{};
    std::vector<uint8_t> active_channels_{};
    std::array<ChannelConfig, kNumChannels> channel_configs_{};

    // Tigertoe channel (can2, standard CAN, independent of Damiao)
    std::unique_ptr<TigertoeChannel> tigertoe_channel_;

    double control_rate_hz_{200.0};
    bool running_{false};
};

}  // namespace w3_robot_bridge
