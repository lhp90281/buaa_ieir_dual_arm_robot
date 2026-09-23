#include "w3_robot_bridge/motor_config.hpp"
#include "w3_robot_bridge/tigertoe_channel.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

#include <cstdio>
#include <string>

namespace w3_robot_bridge {

namespace {

/// Substitute $(find-pkg-share <pkg>) with the actual package share path.
std::string substitute_pkg_share(const std::string& in) {
    const std::string token = "$(find-pkg-share ";
    const auto begin = in.find(token);
    if (begin == std::string::npos) return in;
    const auto end = in.find(")", begin);
    if (end == std::string::npos) return in;
    const std::string pkg = in.substr(begin + token.size(), end - (begin + token.size()));
    // Trim whitespace
    std::string share;
    try {
        share = ament_index_cpp::get_package_share_directory(pkg);
    } catch (...) {
        return in;
    }
    return in.substr(0, begin) + share + in.substr(end + 1);
}

/// Apply YAML fields to a MotorProfile (override defaults with config values).
void apply_yaml_to_profile(const YAML::Node& node, MotorProfile& profile) {
    // --- ranges ---
    if (auto r = node["ranges"]; r) {
        if (auto v = r["position"]; v) {
            profile.pos_range.min = v["min"].as<float>(profile.pos_range.min);
            profile.pos_range.max = v["max"].as<float>(profile.pos_range.max);
        }
        if (auto v = r["velocity"]; v) {
            profile.vel_range.min = v["min"].as<float>(profile.vel_range.min);
            profile.vel_range.max = v["max"].as<float>(profile.vel_range.max);
        }
        if (auto v = r["kp"]; v) {
            profile.kp_range.min = v["min"].as<float>(profile.kp_range.min);
            profile.kp_range.max = v["max"].as<float>(profile.kp_range.max);
        }
        if (auto v = r["kd"]; v) {
            profile.kd_range.min = v["min"].as<float>(profile.kd_range.min);
            profile.kd_range.max = v["max"].as<float>(profile.kd_range.max);
        }
        if (auto v = r["torque"]; v) {
            profile.tor_range.min = v["min"].as<float>(profile.tor_range.min);
            profile.tor_range.max = v["max"].as<float>(profile.tor_range.max);
        }
    }

    // --- limits ---
    if (auto l = node["limits"]; l) {
        if (auto v = l["position"]; v) {
            profile.limits.position_min = v["min"].as<float>(profile.limits.position_min);
            profile.limits.position_max = v["max"].as<float>(profile.limits.position_max);
        }
        profile.limits.velocity_max = l["velocity_max"].as<float>(profile.limits.velocity_max);
        profile.limits.kp_max       = l["kp_max"].as<float>(profile.limits.kp_max);
        profile.limits.kd_max       = l["kd_max"].as<float>(profile.limits.kd_max);
        profile.limits.torque_max   = l["torque_max"].as<float>(profile.limits.torque_max);
    }

    // --- signs ---
    if (auto s = node["signs"]; s) {
        profile.signs.position = s["position"].as<float>(profile.signs.position);
        profile.signs.velocity = s["velocity"].as<float>(profile.signs.velocity);
        profile.signs.torque   = s["torque"].as<float>(profile.signs.torque);
    }

    // --- position_offset ---
    if (auto o = node["position_offset"]; o) {
        profile.position_offset = o.as<float>(profile.position_offset);
    }

    // --- motor identification ---
    if (auto m = node["motor_model"]; m) {
        profile.model_name = m.as<std::string>();
    }
    if (auto c = node["can_id"]; c) {
        profile.can_id = c.as<uint16_t>(profile.can_id);
    }
    if (auto a = node["active"]; a) {
        profile.active = a.as<bool>(profile.active);
    }

    // --- feedback_type (optional, for parameterized feedback decoding) ---
    if (auto ft = node["feedback_type"]; ft) {
        profile.feedback_type = ft.as<uint8_t>(profile.feedback_type);
    }
}

}  // anonymous namespace

// ── Public API ─────────────────────────────────────────────────

bool load_motor_profile(const std::string& yaml_path, MotorProfile& profile) {
    try {
        YAML::Node node = YAML::LoadFile(yaml_path);
        apply_yaml_to_profile(node, profile);
        return true;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "[w3_bridge] failed to load motor config %s: %s\n",
                     yaml_path.c_str(), e.what());
        return false;
    }
}

bool load_bridge_config(const std::string& yaml_path,
                        std::array<ChannelConfig, kNumChannels>& channels,
                        std::unique_ptr<TigertoeChannel>* tigertoe_out) {
    try {
        YAML::Node root = YAML::LoadFile(yaml_path);

        auto ch_node = root["channels"];
        if (!ch_node || !ch_node.IsSequence()) {
            std::fprintf(stderr, "[w3_bridge] config missing 'channels' sequence\n");
            return false;
        }

        for (size_t i = 0; i < ch_node.size() && i < kNumChannels; ++i) {
            auto ch = ch_node[i];
            channels[i].channel_index = static_cast<uint8_t>(i);

            channels[i].iface = ch["iface"].as<std::string>("");
            if (channels[i].iface.empty()) {
                channels[i].motor_count = 0;
                continue;
            }

            // Check for tigertoe protocol (separate from Damiao)
            std::string protocol = ch["protocol"].as<std::string>("damiao");
            if (protocol == "tigertoe" && tigertoe_out) {
                // ── Create TigertoeChannel ──────────────────────
                std::string iface  = channels[i].iface;
                uint32_t baudrate  = ch["baudrate"].as<uint32_t>(1000000);
                auto cfg_paths_node = ch["motor_configs"];

                std::vector<std::string> cfg_paths;
                if (cfg_paths_node && cfg_paths_node.IsSequence()) {
                    for (size_t m = 0; m < cfg_paths_node.size(); ++m) {
                        cfg_paths.push_back(cfg_paths_node[m].as<std::string>(""));
                    }
                }

                std::vector<TigertoeMotorConfig> profiles;
                load_tigertoe_configs(cfg_paths, profiles);

                auto chan = std::make_unique<TigertoeChannel>();
                if (!chan->open(iface, baudrate, profiles)) {
                    std::fprintf(stderr, "[w3_bridge] tigertoe: failed to open %s — skipping\n",
                                 iface.c_str());
                    channels[i].motor_count = 0;
                    continue;  // non-fatal: damiao channels still work
                }
                *tigertoe_out = std::move(chan);

                // Mark as empty in damiao channels (tigertoe handled separately)
                channels[i].motor_count = 0;
                continue;
            }

            // ── Standard Damiao channel loading ────────────────
            channels[i].motor_count        = ch["motor_count"].as<uint8_t>(0);
            channels[i].baudrate           = ch["baudrate"].as<uint32_t>(kDefaultBaudrate);
            channels[i].data_baudrate      = ch["data_baudrate"].as<uint32_t>(kDefaultDataBaud);
            channels[i].control_rate_hz    = ch["control_rate_hz"].as<double>(200.0);
            channels[i].feedback_timeout_s = ch["feedback_timeout_s"].as<double>(0.02);

            auto motor_configs = ch["motor_configs"];
            if (motor_configs && motor_configs.IsSequence()) {
                channels[i].profiles.clear();
                for (size_t m = 0; m < motor_configs.size(); ++m) {
                    MotorProfile profile;
                    profile.channel     = static_cast<uint8_t>(i);
                    profile.motor_index = static_cast<uint8_t>(m);
                    std::string cfg_path = substitute_pkg_share(
                        motor_configs[m].as<std::string>(""));
                    if (!cfg_path.empty()) {
                        if (!load_motor_profile(cfg_path, profile)) {
                            std::fprintf(stderr,
                                "[w3_bridge] WARN: failed to load motor config %s, using defaults\n",
                                cfg_path.c_str());
                        }
                    }
                    channels[i].profiles.push_back(profile);
                }
            }
        }

        return true;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "[w3_bridge] failed to load bridge config %s: %s\n",
                     yaml_path.c_str(), e.what());
        return false;
    }
}

// ── Tigertoe (钛虎) config loading ──────────────────────────────

bool load_tigertoe_motor_profile(const std::string& yaml_path,
                                 TigertoeMotorConfig& cfg) {
    try {
        YAML::Node node = YAML::LoadFile(yaml_path);

        if (auto v = node["can_id"]; v)        cfg.can_id       = v.as<uint16_t>(cfg.can_id);
        if (auto v = node["active"]; v)         cfg.active       = v.as<bool>(cfg.active);
        if (auto v = node["zero_offset"]; v)    cfg.zero_offset  = v.as<int32_t>(cfg.zero_offset);
        if (auto v = node["gear_ratio"]; v)     cfg.gear_ratio   = v.as<float>(cfg.gear_ratio);
        if (auto v = node["direction"]; v)      cfg.direction    = v.as<int8_t>(cfg.direction);
        if (auto v = node["position_min"]; v)   cfg.position_min = v.as<float>(cfg.position_min);
        if (auto v = node["position_max"]; v)   cfg.position_max = v.as<float>(cfg.position_max);

        return true;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "[w3_bridge:tigertoe] failed to load %s: %s\n",
                     yaml_path.c_str(), e.what());
        return false;
    }
}

bool load_tigertoe_configs(const std::vector<std::string>& yaml_paths,
                           std::vector<TigertoeMotorConfig>& profiles) {
    profiles.clear();
    for (const auto& path : yaml_paths) {
        std::string resolved = substitute_pkg_share(path);
        TigertoeMotorConfig cfg;
        if (!resolved.empty()) {
            load_tigertoe_motor_profile(resolved, cfg);
        }
        profiles.push_back(cfg);
    }
    return true;
}

}  // namespace w3_robot_bridge
