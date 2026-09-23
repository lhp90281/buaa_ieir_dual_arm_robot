#pragma once

#include "w3_robot_bridge/motor_types.hpp"

#include <cstdint>

namespace w3_robot_bridge::protocol {

// ── Float ↔ uint conversion (command encoding & feedback decoding) ──
uint16_t float_to_u16(float val, float lo, float hi);
float u16_to_float(uint16_t val, float lo, float hi);
uint16_t float_to_u12(float val, float lo, float hi);
float u12_to_float(uint16_t val, float lo, float hi);

// ── Safety limit application ───────────────────────────────────
void apply_limits(MotorCommand& cmd, const SafetyLimits& lim);

// ── MIT mode command encoding ───────────────────────────────────
/// Per official damiao.cpp control_mit():
///   CAN ID = motor CAN ID (MIT_MODE=0x000)
///   D[0]=pos[15:8]  D[1]=pos[7:0]
///   D[2]=vel[11:4]  D[3]=vel[3:0]|kp[11:8]
///   D[4]=kp[7:0]    D[5]=kd[11:4]
///   D[6]=kd[3:0]|tau[11:8]  D[7]=tau[7:0]
void encode_mit_command(const MotorCommand& cmd,
                        const MotorProfile&  profile,
                        uint8_t              can_data[8]);

// ── Feedback decoding ───────────────────────────────────────────
/// Raw CAN feedback frame (SocketCAN, no USB adapter header):
///   data[0] = motor ID (low nibble), status (high nibble)
///   data[1:2] = Position (uint16, big-endian)
///   data[3]+data[4][7:4] = Velocity (uint12, big-endian packed)
///   data[4][3:0]+data[5] = Torque   (uint12, big-endian packed)
///
/// All values use uint range mapping.
/// Feedback CAN ID is the configured host ID, including shared ID 0.
bool decode_feedback(const uint8_t       data[8],
                     const MotorProfile& profile,
                     MotorState&         out);

}  // namespace w3_robot_bridge::protocol
