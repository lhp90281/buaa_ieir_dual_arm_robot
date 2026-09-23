#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

namespace w3_robot_bridge {

// ── Global constants ──────────────────────────────────────────
constexpr uint8_t  kNumChannels       = 4;       // can0..can3 (2 active + 2 reserved)
constexpr uint16_t kBroadcastCanId    = 0x7FF;   // broadcast/config CAN ID
constexpr uint8_t  kCanFrameLen       = 8;       // MIT mode always uses 8 bytes
constexpr uint32_t kDefaultBaudrate   = 1000000; // 1 Mbps arbitration
constexpr uint32_t kDefaultDataBaud   = 5000000; // 5 Mbps data phase (CAN FD)

// Motor counts per channel (index = channel number)
constexpr uint8_t kMotorsPerChannel[kNumChannels] = {
    8,  // can0 — left arm
    8,  // can1 — right arm
    0,  // can2 — reserved
    0   // can3 — reserved
};

// ── Enums ──────────────────────────────────────────────────────
enum class MotorMode : uint8_t {
    DISABLE = 0,  // Stop sending commands (motor internal watchdog takes over)
    ENABLE  = 1,  // Zero-torque enable (heartbeat starts, motor holds position)
    ZERO    = 2,  // Set current position as zero (via broadcast CAN frame)
    RUN     = 3,  // Force-position hybrid control (MIT mode)
    DAMP    = 4,  // Damping brake (protection mode): kp=0, kd>0, all others zero
};

// Damiao motor error flags (from feedback frame error field)
enum class MotorError : uint8_t {
    NONE              = 0,
    OVERHEAT          = 1,
    OVERCURRENT       = 2,
    UNDERVOLTAGE      = 3,
    ENCODER_ERROR     = 4,
    BRAKE_OVERVOLTAGE = 6,
    DRV_ERROR         = 7,
};

// ── Protocol range mapping (uint ↔ physical value) ──────────
struct ProtocolRange {
    float min{0.0f};
    float max{0.0f};
};

// ── Safety limits (applied BEFORE encoding) ────────────────────
struct SafetyLimits {
    float position_min{-3.1416f};
    float position_max{ 3.1416f};
    float velocity_max{30.0f};
    float kp_max{500.0f};
    float kd_max{5.0f};
    float torque_max{10.0f};
};

// ── Axis sign inversion (+1.0 or -1.0) ────────────────────────
struct AxisSigns {
    float position{1.0f};
    float velocity{1.0f};
    float torque{1.0f};
};

// ── Per-motor configuration profile ────────────────────────────
struct MotorProfile {
    std::string  model_name;       // "DM4310", "DM4340" etc.
    uint16_t     can_id{1};        // Motor CAN ID (1~0x7FE)
    uint8_t      channel{0};       // Which CAN bus (0~3)
    uint8_t      motor_index{0};   // Slot index on that bus

    // Protocol ranges — define uint ↔ physical mapping
    // Defaults per Damiao MIT mode spec: pos ±12.5rad, vel ±30rad/s,
    // kp 0~500, kd 0~5, torque ±10Nm
    ProtocolRange pos_range{-12.5f, 12.5f};
    ProtocolRange vel_range{-30.0f, 30.0f};
    ProtocolRange kp_range{0.0f, 500.0f};
    ProtocolRange kd_range{0.0f, 5.0f};
    ProtocolRange tor_range{-10.0f, 10.0f};

    // Safety limits
    SafetyLimits limits{};

    // Zero point compensation (rad) — aligns model zero with physical zero
    float position_offset{0.0f};

    // Axis sign inversion — set to -1.0 to flip direction
    AxisSigns signs{};

    // Whether this slot is actually used
    bool active{true};

    // Feedback frame type for parameterized decoding:
    //   0 = packed format (mirrors command layout, uint12 fields)
    //   1 = 16-bit aligned int16, little-endian (DM4310/DM4340 default)
    uint8_t feedback_type{1};
};

// ── Desired motor command ─────────────────────────────────────
struct MotorCommand {
    MotorMode mode{MotorMode::DISABLE};
    float position{0.0f};   // rad
    float velocity{0.0f};   // rad/s
    float kp{0.0f};         // position gain
    float kd{0.0f};         // velocity gain
    float torque{0.0f};     // Nm feed-forward
};

// ── Motor feedback / state ────────────────────────────────────
struct MotorState {
    uint16_t can_id{0};
    float    position{0.0f};     // rad (after sign + offset compensation)
    float    velocity{0.0f};     // rad/s
    float    torque{0.0f};       // Nm
    uint8_t  error_flags{0};     // error bitmask
    bool     enabled{false};     // tracked on TX command side
    bool     online{false};      // true if feedback received within timeout
    std::chrono::steady_clock::time_point last_update{};
};

// ── CAN channel configuration ──────────────────────────────────
struct ChannelConfig {
    std::string                iface;              // e.g. "can0"
    uint8_t                    channel_index{0};
    uint8_t                    motor_count{0};
    uint32_t                   baudrate{kDefaultBaudrate};
    uint32_t                   data_baudrate{kDefaultDataBaud};  // CAN FD data phase rate
    double                     control_rate_hz{200.0};           // per-motor rate
    double                     feedback_timeout_s{0.05};         // 50ms for 1kHz feedback
    std::vector<MotorProfile>  profiles;                         // one per motor on this channel
};

// ── Utility helpers ────────────────────────────────────────────
inline const char* mode_name(MotorMode m) {
    switch (m) {
        case MotorMode::DISABLE: return "DISABLE";
        case MotorMode::ENABLE:  return "ENABLE";
        case MotorMode::ZERO:    return "ZERO";
        case MotorMode::RUN:     return "RUN";
        case MotorMode::DAMP:    return "DAMP";
    }
    return "?";
}

inline const char* error_name(uint8_t e) {
    switch (e) {
        case 0: return "none";
        case 1: return "overheat";
        case 2: return "overcurrent";
        case 3: return "undervoltage";
        case 4: return "encoder";
        case 6: return "brake_overvoltage";
        case 7: return "drv_error";
    }
    return "?";
}

// ── Tigertoe (钛虎) encoder motor config ─────────────────────
// Separate from Damiao MIT motors — different protocol, different config.

struct TigertoeMotorConfig {
    uint16_t can_id{1};
    int32_t  zero_offset{0};          // encoder zero compensation (cnt)
    float    gear_ratio{1.0f};        // gear reduction ratio
    int8_t   direction{1};            // rotation direction: +1 / -1
    float    position_min{-6.2832f};  // position lower limit (rad)
    float    position_max{ 6.2832f};  // position upper limit (rad)
    bool     active{true};
};

// Tigertoe motor runtime state
struct TigertoeMotorState {
    int32_t  position_cnt{0};      // latest encoder position (cnt)
    float    position_rad{0.0f};   // converted angle (rad)
    bool     online{false};
    std::chrono::steady_clock::time_point last_update{};
};

}  // namespace w3_robot_bridge
