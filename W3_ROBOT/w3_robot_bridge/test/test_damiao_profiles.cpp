#include "w3_robot_bridge/damiao_protocol.hpp"
#include "w3_robot_bridge/motor_config.hpp"
#include "w3_robot_bridge/can_channel.hpp"
#include <linux/can.h>

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

using namespace w3_robot_bridge;

void require(bool value, const std::string & message) {
  if (!value) throw std::runtime_error(message);
}

int main(int argc, char ** argv) {
  require(argc == 2, "Expected config directory");
  std::unordered_map<uint16_t, uint8_t> slots;
  for (uint8_t id = 1; id <= 7; ++id) {
    slots[id] = id - 1;
    slots[id + 0x10] = id - 1;
  }
  for (uint8_t id = 1; id <= 7; ++id) {
    for (uint8_t status : {0, 1, 10}) {
      uint8_t frame[8] = {static_cast<uint8_t>((status << 4) | id)};
      for (uint32_t host : {0u, static_cast<uint32_t>(id + 0x10)}) {
        const auto slot = resolve_feedback_slot(host, frame, 8, slots);
        require(slot && *slot == id - 1, "Feedback host ID routing failed");
      }
      require(!resolve_feedback_slot(CAN_ERR_FLAG, frame, 8, slots), "Accepted error frame");
      require(!resolve_feedback_slot(CAN_RTR_FLAG, frame, 8, slots), "Accepted remote frame");
      require(!resolve_feedback_slot(0, frame, 7, slots), "Accepted short frame");
      require(!resolve_feedback_slot(0x7ff, frame, 8, slots), "Accepted broadcast");
    }
  }
  uint8_t unknown[8] = {0x18};
  require(!resolve_feedback_slot(0, unknown, 8, slots), "Accepted absent gripper");
  unknown[0] = 0;
  require(!resolve_feedback_slot(0, unknown, 8, slots), "Accepted unknown motor");
  for (int channel = 0; channel < 2; ++channel) {
    const std::string side = channel == 0 ? "left" : "right";
    for (int id = 1; id <= 8; ++id) {
      const float vmax = id <= 2 ? 45.0f : id <= 4 ? 20.0f : 50.0f;
      const float tmax = id <= 2 ? 54.0f : id <= 4 ? 28.0f : 10.0f;
      const std::string path = std::string(argv[1]) + "/motor_config/" +
        side + "_arm/can" + std::to_string(channel) + "_id" + std::to_string(id) + ".yaml";
      MotorProfile profile;
      require(load_motor_profile(path, profile), path);
      MotorCommand command;
      command.mode = MotorMode::RUN;
      command.position = 4.0f;  // raw calibration offsets can place zero beyond pi
      command.velocity = vmax;
      command.torque = tmax;
      command.kp = 0.0f;
      command.kd = 0.0f;
      protocol::apply_limits(command, profile.limits);
      require(command.position == 4.0f, "Raw angle was incorrectly clamped at pi");
      uint8_t bytes[8];
      protocol::encode_mit_command(command, profile, bytes);
      require(bytes[2] == 0xff && (bytes[3] >> 4) == 0xf, "Velocity scale mismatch");
      require((bytes[6] & 0xf) == 0xf && bytes[7] == 0xff, "Torque scale mismatch");
      // Known MIT feedback: midpoint position, maximum velocity, minimum torque.
      uint8_t feedback[8] = {static_cast<uint8_t>(0x10 | id), 0x80, 0x00,
                            0xff, 0xf0, 0x00, 25, 25};
      MotorState state;
      require(protocol::decode_feedback(feedback, profile, state), "Decode failed");
      require(state.enabled && state.error_flags == 1, "Status lost");
      require(std::abs(state.position) < 0.001f, "Position field shifted");
      require(std::abs(state.velocity - vmax) < 0.001f, "Feedback velocity mismatch");
      require(std::abs(state.torque + tmax) < 0.001f, "Feedback torque mismatch");
      feedback[0] = static_cast<uint8_t>(0xa0 | id);
      protocol::decode_feedback(feedback, profile, state);
      require(!state.enabled && state.error_flags == 10, "Motor fault lost");
    }
  }
  std::cout << "All 16 motor profiles passed MIT command/feedback checks\n";
}
