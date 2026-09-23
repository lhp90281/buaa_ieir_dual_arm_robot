#pragma once

#include <cstdint>

namespace w3_robot_bridge::tigertoe {

// ── Encoder cnt ↔ rad conversion ──────────────────────────────
/// Convert encoder counts to radians.
///   angle = (cnt - zero_offset) / gear_ratio * direction
///   direction = -1 flips the rotation sign.
float cnt_to_rad(int32_t cnt, int32_t zero_offset, float gear_ratio, int direction);

/// Convert radians to encoder counts (reverse of cnt_to_rad).
int32_t rad_to_cnt(float rad, int32_t zero_offset, float gear_ratio, int direction);

// ── Command encoding ──────────────────────────────────────────

/// Brake command: {0x02}  1 byte, no reply.
void encode_brake(uint8_t data[1]);

/// Get-position command: {0x08}  1 byte, motor replies 5 bytes.
void encode_get_position(uint8_t data[1]);

/// Set-position command: {0x1E, cnt[0:3]}  5 bytes, no reply.
/// cnt is int32 little-endian.
void encode_set_position(int32_t target_cnt, uint8_t data[5]);

// ── Reply decoding ────────────────────────────────────────────

/// Decode position reply: data={0x08, cnt[0:3]}, 5 bytes int32 LE.
/// Returns false if data[0] != 0x08.
bool decode_position_reply(const uint8_t data[5], int32_t& out_cnt);

}  // namespace w3_robot_bridge::tigertoe
