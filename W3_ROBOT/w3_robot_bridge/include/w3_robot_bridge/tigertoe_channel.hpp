#pragma once

#include "w3_robot_bridge/motor_types.hpp"

#include <atomic>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <unordered_map>

namespace w3_robot_bridge {

/// Tigertoe (钛虎) encoder motor CAN channel.
///
/// Completely independent of CanChannel (Damiao MIT).
/// Uses standard CAN 2.0 (not FD), one-shot command-response model.
///
/// Thread model:
///   - NO TX thread — brake/get/set sent directly from calling thread
///   - RX thread — blocking read, dispatches position replies
class TigertoeChannel {
public:
    TigertoeChannel() = default;
    ~TigertoeChannel();

    TigertoeChannel(const TigertoeChannel&)            = delete;
    TigertoeChannel& operator=(const TigertoeChannel&) = delete;

    /// Open standard CAN socket, bind to interface.
    bool open(const std::string& iface, uint32_t baudrate,
              const std::vector<TigertoeMotorConfig>& profiles);

    /// Start RX thread.
    bool start();

    /// Stop RX thread, close socket.
    void stop();

    // ── One-shot commands (calling thread, no TX loop) ───────
    /// Send brake command (0x02) to a motor. No reply.
    void sendBrake(uint8_t motor_index);

    /// Send get-position command (0x08). Reply handled by RX thread.
    void sendGetPosition(uint8_t motor_index);

    /// Send set-position command (0x1E + int32 cnt). No reply.
    void sendSetPosition(uint8_t motor_index, int32_t target_cnt);

    // ── State access ─────────────────────────────────────────
    TigertoeMotorState getState(uint8_t motor_index) const;
    bool isOnline(uint8_t motor_index) const;

    /// Mark motors offline if no feedback within timeout. Call periodically.
    void refreshOnlineFlags();

    // ── Accessors ────────────────────────────────────────────
    uint8_t motorCount() const { return cfg_.motor_count; }
    const std::string& iface() const { return iface_; }
    const std::vector<TigertoeMotorConfig>& profiles() const { return profiles_; }

private:
    bool validIndex(uint8_t motor_index) const {
        return motor_index < cfg_.motor_count;
    }

    void rxLoop();

    int  sock_{-1};
    std::string iface_;

    struct ChannelCfg {
        uint32_t baudrate{1000000};
        uint8_t  motor_count{0};
        double   feedback_timeout_s{0.5};
    } cfg_;

    std::vector<TigertoeMotorConfig> profiles_;
    std::vector<TigertoeMotorState>  states_;
    std::unordered_map<uint16_t, uint8_t> can_id_to_slot_;  // CAN ID → slot index

    std::thread rx_thread_;
    std::atomic<bool> running_{false};
    mutable std::mutex state_mutex_;
};

}  // namespace w3_robot_bridge
