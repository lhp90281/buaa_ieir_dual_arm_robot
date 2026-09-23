#include "w3_robot_bridge/damiao_protocol.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace w3_robot_bridge::protocol {

// ── Float ↔ uint conversion (for COMMAND encoding) ──────────────

uint16_t float_to_u16(float val, float lo, float hi) {
    if (lo >= hi) return 0;
    val = std::clamp(val, lo, hi);
    return static_cast<uint16_t>((val - lo) / (hi - lo) * 65535.0f + 0.5f);
}

float u16_to_float(uint16_t val, float lo, float hi) {
    if (lo >= hi) return lo;
    return static_cast<float>(val) * (hi - lo) / 65535.0f + lo;
}

uint16_t float_to_u12(float val, float lo, float hi) {
    if (lo >= hi) return 0;
    val = std::clamp(val, lo, hi);
    return static_cast<uint16_t>((val - lo) / (hi - lo) * 4095.0f + 0.5f);
}

float u12_to_float(uint16_t val, float lo, float hi) {
    if (lo >= hi) return lo;
    return static_cast<float>(val) * (hi - lo) / 4095.0f + lo;
}

// ── Signed int ↔ float conversion (for FEEDBACK decoding) ──────
//
// Per DM-J4310-2EC manual: "位置、速度和扭矩采用线性映射的关系
// 将浮点型数据转换成有符号的定点数据，其中位置采用 16 位数据，
// 速度和扭矩均使用 12 位"
//
// Position: signed int16 [-32768, 32767] → [P_MIN, P_MAX]
// Velocity: signed int12 [-2048,  2047]  → [V_MIN, V_MAX]
// Torque:   signed int12 [-2048,  2047]  → [T_MIN, T_MAX]

float s16_to_float(int16_t val, float lo, float hi) {
    if (lo >= hi) return lo;
    // val=32767 → hi, val=0 → (lo+hi)/2, val=-32768 → lo
    float norm = (static_cast<float>(val) + 32768.0f) / 65535.0f;
    return lo + norm * (hi - lo);
}

int16_t float_to_s16(float val, float lo, float hi) {
    if (lo >= hi) return 0;
    val = std::clamp(val, lo, hi);
    float norm = (val - lo) / (hi - lo);
    return static_cast<int16_t>(norm * 65535.0f - 32768.0f + 0.5f);
}

float s12_to_float(int16_t val, float lo, float hi) {
    if (lo >= hi) return lo;
    // val is 12-bit signed: [-2048, 2047]
    // val=2047 → hi, val=0 → (lo+hi)/2, val=-2048 → lo
    float norm = (static_cast<float>(val) + 2048.0f) / 4095.0f;
    return lo + norm * (hi - lo);
}

int16_t float_to_s12(float val, float lo, float hi) {
    if (lo >= hi) return 0;
    val = std::clamp(val, lo, hi);
    float norm = (val - lo) / (hi - lo);
    return static_cast<int16_t>(norm * 4095.0f - 2048.0f + 0.5f);
}

// ── Safety limits ──────────────────────────────────────────────

void apply_limits(MotorCommand& cmd, const SafetyLimits& lim) {
    if (cmd.mode != MotorMode::RUN && cmd.mode != MotorMode::DAMP) return;
    cmd.position = std::clamp(cmd.position, lim.position_min, lim.position_max);
    cmd.velocity = std::clamp(cmd.velocity, -lim.velocity_max, lim.velocity_max);
    cmd.kp       = std::clamp(cmd.kp,       0.0f,             lim.kp_max);
    cmd.kd       = std::clamp(cmd.kd,       0.0f,             lim.kd_max);
    cmd.torque   = std::clamp(cmd.torque,   -lim.torque_max,  lim.torque_max);
}

// ── MIT mode command encoding ───────────────────────────────────
//
// Per DM-J4310-2EC manual §MIT模式:
//   CAN ID = motor CAN ID (no offset, 11-bit standard frame)
//   8-byte layout (big-endian, 12-bit packed fields):
//     D[0] : p_des[15:8]
//     D[1] : p_des[7:0]
//     D[2] : v_des[11:4]
//     D[3] : v_des[3:0] | Kp[11:8]
//     D[4] : Kp[7:0]
//     D[5] : Kd[11:4]
//     D[6] : Kd[3:0] | t_ff[11:8]
//     D[7] : t_ff[7:0]
//
// Ranges: p_des/v_des/t_ff set by debug tool; Kp=[0,500]; Kd=[0,5]

void encode_mit_command(const MotorCommand& cmd,
                        const MotorProfile&  profile,
                        uint8_t              can_data[8]) {
    if (cmd.mode != MotorMode::RUN && cmd.mode != MotorMode::DAMP) {
        std::memset(can_data, 0, 8);
        return;
    }

    const bool is_damp = (cmd.mode == MotorMode::DAMP);

    float pos_mapped, vel_mapped, tor_mapped;
    float kp_val, kd_val;

    if (is_damp) {
        pos_mapped = 0.0f;
        vel_mapped = 0.0f;
        tor_mapped = 0.0f;
        kp_val     = 0.0f;
        kd_val     = cmd.kd;
    } else {
        pos_mapped = (cmd.position - profile.position_offset) * profile.signs.position;
        vel_mapped = cmd.velocity * profile.signs.velocity;
        tor_mapped = cmd.torque * profile.signs.torque;
        kp_val     = cmd.kp;
        kd_val     = cmd.kd;
    }

    uint16_t pos_u16 = float_to_u16(pos_mapped,
                                     profile.pos_range.min, profile.pos_range.max);
    uint16_t vel_u12 = float_to_u12(vel_mapped,
                                     profile.vel_range.min, profile.vel_range.max);
    uint16_t kp_u12  = float_to_u12(kp_val,
                                     profile.kp_range.min, profile.kp_range.max);
    uint16_t kd_u12  = float_to_u12(kd_val,
                                     profile.kd_range.min, profile.kd_range.max);
    uint16_t tor_u12 = float_to_u12(tor_mapped,
                                     profile.tor_range.min, profile.tor_range.max);

    can_data[0] = static_cast<uint8_t>((pos_u16 >> 8) & 0xFF);
    can_data[1] = static_cast<uint8_t>(pos_u16 & 0xFF);
    can_data[2] = static_cast<uint8_t>((vel_u12 >> 4) & 0xFF);
    can_data[3] = static_cast<uint8_t>(((vel_u12 & 0x0F) << 4)
                                     | ((kp_u12 >> 8) & 0x0F));
    can_data[4] = static_cast<uint8_t>(kp_u12 & 0xFF);
    can_data[5] = static_cast<uint8_t>((kd_u12 >> 4) & 0xFF);
    can_data[6] = static_cast<uint8_t>(((kd_u12 & 0x0F) << 4)
                                     | ((tor_u12 >> 8) & 0x0F));
    can_data[7] = static_cast<uint8_t>(tor_u12 & 0xFF);
}

// ── Feedback decoding ───────────────────────────────────────────
//
// Raw CAN feedback frame from Damiao motor:
//
//   data[0]   : MST_ID / reserved (skipped)
//   data[1:2] : Position (uint16, big-endian)
//   data[3] + data[4][7:4] : Velocity (uint12, big-endian packed)
//   data[4][3:0] + data[5] : Torque   (uint12, big-endian packed)
//   data[6:7] : temperature / status
//
// All values use uint range mapping.
// Feedback CAN ID = mst_id = can_id + 0x10.

namespace {

bool decode_feedback_raw(const uint8_t* data,
                         const MotorProfile& profile,
                         MotorState& out) {
    // ── Per DM-J4310-2EC manual p.6 反馈帧:
    //   D[0]=ID|ERR<<4  D[1]=POS[15:8]  D[2]=POS[7:0]
    //   D[3]=VEL[11:4]  D[4]=VEL[3:0]|T[11:8]
    //   D[5]=T[7:0]     D[6]=T_MOS      D[7]=T_Rotor

    // Error/status from D[0] high nibble
    uint8_t motor_id  = data[0] & 0x0F;
    uint8_t err_code  = (data[0] >> 4) & 0x0F;
    // ERR=0 → 失能, ERR=1 → 使能, ERR≥8 → 故障(退出使能)
    out.error_flags = err_code;
    out.enabled     = (err_code == 1);

    // Position: uint16 @ D[1:2], big-endian
    uint16_t pos_u16 = (static_cast<uint16_t>(data[1]) << 8) | data[2];
    float pos_raw = u16_to_float(pos_u16,
                                  profile.pos_range.min, profile.pos_range.max);

    // Velocity: uint12 @ D[3] + D[4][7:4]
    uint16_t vel_u12 = (static_cast<uint16_t>(data[3]) << 4) | ((data[4] >> 4) & 0x0F);
    float vel_raw = u12_to_float(vel_u12,
                                  profile.vel_range.min, profile.vel_range.max);

    // Torque: uint12 @ D[4][3:0] + D[5]
    uint16_t tor_u12 = (static_cast<uint16_t>(data[4] & 0x0F) << 8) | data[5];
    float tor_raw = u12_to_float(tor_u12,
                                  profile.tor_range.min, profile.tor_range.max);

    // Apply sign inversion + zero offset
    out.position    = pos_raw * profile.signs.position + profile.position_offset;
    out.velocity    = vel_raw * profile.signs.velocity;
    out.torque      = tor_raw * profile.signs.torque;
    out.online      = true;
    out.last_update = std::chrono::steady_clock::now();
    return true;
}

}  // anonymous namespace


// ── Public decode entry point ─────────────────────────────────

bool decode_feedback(const uint8_t       data[8],
                     const MotorProfile& profile,
                     MotorState&         out) {
    if (data == nullptr) return false;
    return decode_feedback_raw(data, profile, out);
}

// ── Special commands ──────────────────────────────────────────

void encode_set_zero(uint16_t motor_can_id, uint8_t payload[4]) {
    payload[0] = static_cast<uint8_t>((motor_can_id >> 8) & 0xFF);
    payload[1] = static_cast<uint8_t>(motor_can_id & 0xFF);
    payload[2] = 0x00;
    payload[3] = 0x03;
}

void encode_set_can_id(uint16_t old_can_id, uint16_t new_can_id,
                       uint8_t payload[6]) {
    payload[0] = static_cast<uint8_t>((old_can_id >> 8) & 0xFF);
    payload[1] = static_cast<uint8_t>(old_can_id & 0xFF);
    payload[2] = 0x00;
    payload[3] = 0x04;
    payload[4] = static_cast<uint8_t>((new_can_id >> 8) & 0xFF);
    payload[5] = static_cast<uint8_t>(new_can_id & 0xFF);
}

}  // namespace w3_robot_bridge::protocol
