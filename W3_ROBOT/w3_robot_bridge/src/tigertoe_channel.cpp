#include "w3_robot_bridge/tigertoe_channel.hpp"
#include "w3_robot_bridge/tigertoe_protocol.hpp"

#include <cstdio>
#include <cstring>
#include <errno.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

namespace w3_robot_bridge {

// ── Lifecycle ───────────────────────────────────────────────────

TigertoeChannel::~TigertoeChannel() {
    stop();
    if (sock_ >= 0) {
        ::close(sock_);
        sock_ = -1;
    }
}

bool TigertoeChannel::open(const std::string& iface, uint32_t baudrate,
                           const std::vector<TigertoeMotorConfig>& profiles) {
    iface_ = iface;
    profiles_ = profiles;
    cfg_.motor_count = static_cast<uint8_t>(profiles.size());
    cfg_.baudrate    = baudrate;

    // --- Create standard CAN socket (NOT FD) ---
    sock_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock_ < 0) {
        std::perror("[w3_bridge:tigertoe] socket(PF_CAN)");
        return false;
    }

    // Disable CAN FD explicitly
    int enable_fd = 0;
    ::setsockopt(sock_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_fd, sizeof(enable_fd));

    // Disable loopback
    int loopback = 0;
    ::setsockopt(sock_, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback));

    int recv_own = 0;
    ::setsockopt(sock_, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &recv_own, sizeof(recv_own));

    // --- Bind to interface ---
    ifreq ifr{};
    std::strncpy(ifr.ifr_name, iface_.c_str(), IFNAMSIZ - 1);
    if (::ioctl(sock_, SIOCGIFINDEX, &ifr) < 0) {
        std::fprintf(stderr, "[w3_bridge:tigertoe] SIOCGIFINDEX %s: %s\n",
                     iface_.c_str(), std::strerror(errno));
        return false;
    }

    sockaddr_can addr{};
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (::bind(sock_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        std::fprintf(stderr, "[w3_bridge:tigertoe] bind(%s): %s\n",
                     iface_.c_str(), std::strerror(errno));
        return false;
    }

    // --- Receive timeout for clean shutdown ---
    timeval tv{};
    tv.tv_sec  = 0;
    tv.tv_usec = 500000;
    ::setsockopt(sock_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    // --- Initialize per-motor state ---
    states_.clear();
    states_.resize(cfg_.motor_count);
    can_id_to_slot_.clear();
    for (uint8_t i = 0; i < cfg_.motor_count; ++i) {
        states_[i] = TigertoeMotorState{};
        can_id_to_slot_[profiles_[i].can_id] = i;
    }

    std::printf("[w3_bridge:tigertoe] Channel %s opened: %u motors, %u bps (std CAN)\n",
                iface_.c_str(), cfg_.motor_count, cfg_.baudrate);
    return true;
}

bool TigertoeChannel::start() {
    if (running_.exchange(true)) return true;
    rx_thread_ = std::thread([this] { rxLoop(); });
    return true;
}

void TigertoeChannel::stop() {
    if (!running_.exchange(false)) return;
    if (sock_ >= 0) ::shutdown(sock_, SHUT_RDWR);
    if (rx_thread_.joinable()) rx_thread_.join();
}

// ── Helper: build standard can_frame from payload ──────────────

static can_frame make_frame(uint16_t can_id, const uint8_t* data, uint8_t len) {
    can_frame frame{};
    frame.can_id  = can_id & CAN_SFF_MASK;
    frame.can_dlc = len;
    std::memcpy(frame.data, data, std::min(static_cast<int>(len), 8));
    return frame;
}

// ── One-shot commands ──────────────────────────────────────────

void TigertoeChannel::sendBrake(uint8_t motor_index) {
    if (!validIndex(motor_index)) return;
    uint8_t payload[1];
    tigertoe::encode_brake(payload);
    auto frame = make_frame(profiles_[motor_index].can_id, payload, 1);
    ::write(sock_, &frame, sizeof(frame));
}

void TigertoeChannel::sendGetPosition(uint8_t motor_index) {
    if (!validIndex(motor_index)) return;
    uint8_t payload[1];
    tigertoe::encode_get_position(payload);
    auto frame = make_frame(profiles_[motor_index].can_id, payload, 1);
    ::write(sock_, &frame, sizeof(frame));
}

void TigertoeChannel::sendSetPosition(uint8_t motor_index, int32_t target_cnt) {
    if (!validIndex(motor_index)) return;
    uint8_t payload[5];
    tigertoe::encode_set_position(target_cnt, payload);
    auto frame = make_frame(profiles_[motor_index].can_id, payload, 5);
    ::write(sock_, &frame, sizeof(frame));
}

// ── State access ───────────────────────────────────────────────

TigertoeMotorState TigertoeChannel::getState(uint8_t motor_index) const {
    if (!validIndex(motor_index)) return {};
    std::lock_guard<std::mutex> lk(state_mutex_);
    return states_[motor_index];
}

bool TigertoeChannel::isOnline(uint8_t motor_index) const {
    if (!validIndex(motor_index)) return false;
    std::lock_guard<std::mutex> lk(state_mutex_);
    return states_[motor_index].online;
}

// ── RX Thread ────────────────────────────────────────────────

void TigertoeChannel::rxLoop() {
    while (running_.load(std::memory_order_acquire)) {
        can_frame frame{};
        const ssize_t n = ::read(sock_, &frame, sizeof(frame));
        if (n < 0) {
            if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) continue;
            break;
        }
        if (n < static_cast<ssize_t>(sizeof(can_frame))) continue;

        uint16_t can_id = frame.can_id & CAN_SFF_MASK;
        if (can_id == 0) continue;

        // Look up slot
        auto it = can_id_to_slot_.find(can_id);
        if (it == can_id_to_slot_.end()) continue;

        uint8_t slot = it->second;
        const auto& cfg = profiles_[slot];

        // Try to decode position reply (5 bytes: 0x08 + int32 LE)
        if (frame.can_dlc >= 5) {
            int32_t pos_cnt = 0;
            bool ok = tigertoe::decode_position_reply(frame.data, pos_cnt);
            if (ok) {
                std::lock_guard<std::mutex> lk(state_mutex_);
                states_[slot].position_cnt = pos_cnt;
                states_[slot].position_rad = tigertoe::cnt_to_rad(
                    pos_cnt, cfg.zero_offset, cfg.gear_ratio, cfg.direction);
                states_[slot].online      = true;
                states_[slot].last_update = std::chrono::steady_clock::now();
            }
        }
    }
}

// ── Online detection ───────────────────────────────────────────

void TigertoeChannel::refreshOnlineFlags() {
    using clock = std::chrono::steady_clock;
    const auto now = clock::now();
    const auto cutoff = std::chrono::duration_cast<clock::duration>(
        std::chrono::duration<double>(cfg_.feedback_timeout_s));

    std::lock_guard<std::mutex> lk(state_mutex_);
    for (auto& s : states_) {
        if (s.last_update.time_since_epoch().count() == 0) {
            s.online = false;
            continue;
        }
        s.online = (now - s.last_update) <= cutoff;
    }
}

}  // namespace w3_robot_bridge
