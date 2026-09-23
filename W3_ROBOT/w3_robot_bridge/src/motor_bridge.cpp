#include "w3_robot_bridge/motor_bridge.hpp"
#include "w3_robot_bridge/motor_config.hpp"

#include <cstdio>
#include <memory>

namespace w3_robot_bridge {

MotorBridge::~MotorBridge() {
    close();
}

bool MotorBridge::init(const std::string& config_path) {
    // Load channel configurations from YAML
    if (!load_bridge_config(config_path, channel_configs_, &tigertoe_channel_)) {
        std::fprintf(stderr, "[w3_bridge] MotorBridge: failed to load config from %s\n",
                     config_path.c_str());
        return false;
    }

    active_channels_.clear();

    for (uint8_t ch = 0; ch < kNumChannels; ++ch) {
        const auto& ch_cfg = channel_configs_[ch];
        if (ch_cfg.iface.empty() || ch_cfg.motor_count == 0) {
            continue;  // skip unused/reserved channels
        }

        auto chan = std::make_unique<CanChannel>();
        if (!chan->open(ch_cfg)) {
            std::fprintf(stderr, "[w3_bridge] MotorBridge: failed to open channel %u (%s)\n",
                         ch, ch_cfg.iface.c_str());
            return false;
        }

        channels_[ch] = std::move(chan);
        active_channels_.push_back(ch);
        control_rate_hz_ = ch_cfg.control_rate_hz;
    }

    std::printf("[w3_bridge] MotorBridge: initialized %zu damiao channel(s) @ %.0f Hz/motor\n",
                active_channels_.size(), control_rate_hz_);

    if (tigertoe_channel_) {
        std::printf("[w3_bridge] MotorBridge: tigertoe channel ready (%s)\n",
                    tigertoe_channel_->iface().c_str());
    }
    return true;
}

bool MotorBridge::start() {
    if (running_) return true;

    for (uint8_t ch : active_channels_) {
        if (!channels_[ch]->start()) {
            std::fprintf(stderr, "[w3_bridge] MotorBridge: failed to start channel %u\n", ch);
            return false;
        }
    }

    if (tigertoe_channel_) {
        tigertoe_channel_->start();
    }

    running_ = true;
    std::printf("[w3_bridge] MotorBridge: all channels started\n");
    return true;
}

void MotorBridge::stop() {
    if (!running_) return;
    for (uint8_t ch : active_channels_) {
        channels_[ch]->stop();
    }
    if (tigertoe_channel_) {
        tigertoe_channel_->stop();
    }
    running_ = false;
}

void MotorBridge::close() {
    stop();
    for (auto& c : channels_) {
        c.reset();
    }
    tigertoe_channel_.reset();
    active_channels_.clear();
}

// ── Command interface ─────────────────────────────────────────

void MotorBridge::setCommand(uint8_t channel, uint8_t motor_index,
                             const MotorCommand& cmd) {
    if (channel < kNumChannels && channels_[channel]) {
        channels_[channel]->setCommand(motor_index, cmd);
    }
}

void MotorBridge::enableMotor(uint8_t channel, uint8_t motor_index) {
    if (channel < kNumChannels && channels_[channel]) {
        channels_[channel]->enableMotor(motor_index);
    }
}

void MotorBridge::disableMotor(uint8_t channel, uint8_t motor_index) {
    if (channel < kNumChannels && channels_[channel]) {
        channels_[channel]->disableMotor(motor_index);
    }
}

void MotorBridge::setZero(uint8_t channel, uint8_t motor_index) {
    if (channel < kNumChannels && channels_[channel]) {
        channels_[channel]->sendSetZero(motor_index);
    }
}

void MotorBridge::enableAll() {
    for (uint8_t ch : active_channels_) {
        const auto& cfg = channel_configs_[ch];
        for (uint8_t i = 0; i < cfg.motor_count; ++i) {
            if (i < cfg.profiles.size() && cfg.profiles[i].active) {
                channels_[ch]->enableMotor(i);
            }
        }
    }
}

void MotorBridge::disableAll() {
    for (uint8_t ch : active_channels_) {
        const auto& cfg = channel_configs_[ch];
        for (uint8_t i = 0; i < cfg.motor_count; ++i) {
            if (i < cfg.profiles.size() && cfg.profiles[i].active) {
                channels_[ch]->disableMotor(i);
            }
        }
    }
}

// ── State access ──────────────────────────────────────────────

MotorState MotorBridge::getState(uint8_t channel, uint8_t motor_index) const {
    if (channel < kNumChannels && channels_[channel]) {
        return channels_[channel]->getState(motor_index);
    }
    return {};
}

bool MotorBridge::isOnline(uint8_t channel, uint8_t motor_index) const {
    if (channel < kNumChannels && channels_[channel]) {
        return channels_[channel]->isOnline(motor_index);
    }
    return false;
}

bool MotorBridge::isEnabled(uint8_t channel, uint8_t motor_index) const {
    if (channel < kNumChannels && channels_[channel]) {
        return channels_[channel]->isEnabled(motor_index);
    }
    return false;
}

const MotorProfile* MotorBridge::getProfile(uint8_t channel, uint8_t motor_index) const {
    if (channel < kNumChannels && channels_[channel]) {
        return channels_[channel]->getProfile(motor_index);
    }
    return nullptr;
}

// ── Channel access ────────────────────────────────────────────

CanChannel* MotorBridge::getChannel(uint8_t ch) {
    if (ch < kNumChannels) return channels_[ch].get();
    return nullptr;
}

const CanChannel* MotorBridge::getChannel(uint8_t ch) const {
    if (ch < kNumChannels) return channels_[ch].get();
    return nullptr;
}

uint8_t MotorBridge::motorCount(uint8_t channel) const {
    if (channel < kNumChannels && channels_[channel]) {
        return channels_[channel]->motorCount();
    }
    return 0;
}

}  // namespace w3_robot_bridge
