#pragma once

#include "w3_robot_bridge/motor_types.hpp"
#include "w3_robot_bridge/damiao_protocol.hpp"

#include <atomic>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>
#include <unordered_map>

namespace w3_robot_bridge {

// Resolve the payload motor ID even when multiple motors share host CAN ID 0.
std::optional<uint8_t> resolve_feedback_slot(
    uint32_t can_id, const uint8_t* data, size_t length,
    const std::unordered_map<uint16_t, uint8_t>& can_id_to_slot);

/// Manages one CAN FD bus interface with multiple Damiao motors.
///
/// Thread model:
///   - TX thread: round-robin sends one CAN FD frame per motor at
///     motor_count × control_rate_hz (1600Hz for 8 motors @ 200Hz).
///     Each motor gets a frame every 5ms = 200Hz.
///   - RX thread: blocking read on socket, dispatches by CAN ID.
///   - Command/state access uses separate mutexes (cmd_mutex_, state_mutex_)
///     to minimize contention between TX and external ROS writers.
///
/// CAN FD specifics:
///   - Uses canfd_frame with CANFD_BRS flag for 5Mbps data phase.
///   - Socket created with CAN_RAW_FD_FRAMES enabled.
class CanChannel {
public:
    CanChannel() = default;
    ~CanChannel();

    CanChannel(const CanChannel&)            = delete;
    CanChannel& operator=(const CanChannel&) = delete;
    CanChannel(CanChannel&&)                 = delete;
    CanChannel& operator=(CanChannel&&)      = delete;

    /// Open the SocketCAN FD interface and configure baudrate.
    /// Must be called before start().
    bool open(const ChannelConfig& cfg);

    /// Start TX and RX threads.
    bool start();

    /// Stop threads and close socket gracefully.
    void stop();

    // ── Thread-safe command interface ──────────────────────────
    /// Set the desired command for a motor (by slot index, 0-based).
    void setCommand(uint8_t motor_index, const MotorCommand& cmd);

    /// Enable a motor (start sending zero-torque heartbeat).
    void enableMotor(uint8_t motor_index);

    /// Disable a motor (stop sending commands, motor watchdog takes over).
    void disableMotor(uint8_t motor_index);

    /// Send a set-zero broadcast command (CAN ID 0x7FF).
    void sendSetZero(uint8_t motor_index);

    // ── Thread-safe state access ────────────────────────────────
    MotorState getState(uint8_t motor_index) const;
    bool isOnline(uint8_t motor_index) const;
    bool isEnabled(uint8_t motor_index) const;
    const MotorProfile* getProfile(uint8_t motor_index) const;

    // ── Statistics ──────────────────────────────────────────────
    size_t txCount() const { return tx_frames_.load(std::memory_order_relaxed); }
    size_t rxCount() const { return rx_frames_.load(std::memory_order_relaxed); }
    uint8_t motorCount() const { return cfg_.motor_count; }
    const std::string& iface() const { return cfg_.iface; }

private:
    bool validIndex(uint8_t motor_index) const {
        return motor_index < cfg_.motor_count;
    }

    void txLoop();
    void rxLoop();
    void refreshOnlineFlags();
    void sendFrame(uint16_t can_id, const uint8_t data[8]);

    int sock_{-1};
    ChannelConfig cfg_{};

    // Per-motor state (indexed by slot)
    std::vector<MotorProfile>  profiles_;
    std::vector<MotorCommand>  commands_;
    std::vector<MotorState>    states_;
    // CAN ID → slot index lookup for RX dispatch
    std::unordered_map<uint16_t, uint8_t> can_id_to_slot_;

    std::thread tx_thread_;
    std::thread rx_thread_;
    std::atomic<bool> running_{false};

    mutable std::mutex cmd_mutex_;    // protects commands_
    mutable std::mutex state_mutex_;  // protects states_

    std::atomic<size_t> tx_frames_{0};
    std::atomic<size_t> rx_frames_{0};
    size_t tx_error_count_{0};  // consecutive TX errors (reset on success)
};

}  // namespace w3_robot_bridge
