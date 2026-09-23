#include "w3_robot_bridge/tigertoe_protocol.hpp"

#include <cstring>

namespace w3_robot_bridge::tigertoe {

// ── Value ↔ rad conversion ───────────────────────────────────
//
// Tigertoe motor uses fixed 262140 counts per motor revolution.
//   Output angle (deg) = (value / 262140 / gear_ratio) * 360
//   Output angle (rad) = (value / 262140 / gear_ratio) * 2π
//   Reverse:
//     value = (target_rad / (2π)) * gear_ratio * 262140 + zero_offset

static constexpr float kCntPerRev = 262140.0f;
static constexpr float kTwoPi     = 6.283185307f;

float cnt_to_rad(int32_t value, int32_t zero_offset, float gear_ratio, int direction) {
    float adjusted = static_cast<float>(value - zero_offset);
    if (gear_ratio > 0.0f) {
        adjusted /= gear_ratio;
    }
    return (adjusted / kCntPerRev) * kTwoPi * static_cast<float>(direction);
}

int32_t rad_to_cnt(float rad, int32_t zero_offset, float gear_ratio, int direction) {
    float adjusted = rad * static_cast<float>(direction);
    float value = (adjusted / kTwoPi) * gear_ratio * kCntPerRev;
    return static_cast<int32_t>(value + 0.5f) + zero_offset;  // round
}

// ── Command encoding ──────────────────────────────────────────

void encode_brake(uint8_t data[1]) {
    data[0] = 0x02;
}

void encode_get_position(uint8_t data[1]) {
    data[0] = 0x08;
}

void encode_set_position(int32_t target_cnt, uint8_t data[5]) {
    data[0] = 0x1E;
    // int32 little-endian
    data[1] = static_cast<uint8_t>(target_cnt & 0xFF);
    data[2] = static_cast<uint8_t>((target_cnt >> 8) & 0xFF);
    data[3] = static_cast<uint8_t>((target_cnt >> 16) & 0xFF);
    data[4] = static_cast<uint8_t>((target_cnt >> 24) & 0xFF);
}

// ── Reply decoding ────────────────────────────────────────────

bool decode_position_reply(const uint8_t data[5], int32_t& out_cnt) {
    if (data[0] != 0x08) return false;

    // int32 little-endian: data[1]=LSB, data[4]=MSB
    out_cnt = static_cast<int32_t>(data[1])
            | (static_cast<int32_t>(data[2]) << 8)
            | (static_cast<int32_t>(data[3]) << 16)
            | (static_cast<int32_t>(data[4]) << 24);
    return true;
}

}  // namespace w3_robot_bridge::tigertoe
