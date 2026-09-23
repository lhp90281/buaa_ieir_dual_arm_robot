#include "w3_robot_bridge/can_channel.hpp"

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

std::optional<uint8_t> resolve_feedback_slot(
    uint32_t can_id, const uint8_t* data, size_t length,
    const std::unordered_map<uint16_t, uint8_t>& can_id_to_slot) {
    if (!data || length < kCanFrameLen || (can_id & (CAN_ERR_FLAG | CAN_RTR_FLAG))) {
        return std::nullopt;
    }
    can_id &= (can_id & CAN_EFF_FLAG) ? CAN_EFF_MASK : CAN_SFF_MASK;
    if (can_id >= kBroadcastCanId) return std::nullopt;

    const uint16_t motor_id = data[0] & 0x0F;
    if (motor_id >= 1 && motor_id <= 8) {
        auto it = can_id_to_slot.find(motor_id);
        if (it != can_id_to_slot.end()) return it->second;
    }
    // Shared host ID alone cannot identify a motor.
    if (can_id == 0) return std::nullopt;
    auto it = can_id_to_slot.find(static_cast<uint16_t>(can_id));
    if (it != can_id_to_slot.end()) return it->second;
    if (can_id >= 0x10) {
        it = can_id_to_slot.find(static_cast<uint16_t>(can_id - 0x10));
        if (it != can_id_to_slot.end()) return it->second;
    }
    return std::nullopt;
}

// ── Lifecycle ───────────────────────────────────────────────────

CanChannel::~CanChannel() {
    stop();
    if (sock_ >= 0) {
        ::close(sock_);
        sock_ = -1;
    }
}

bool CanChannel::open(const ChannelConfig& cfg) {
    cfg_ = cfg;

    // --- Create CAN socket ---
    sock_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock_ < 0) {
        std::perror("[w3_bridge] socket(PF_CAN)");
        return false;
    }

    // --- Enable CAN FD frames (KEY DIFFERENCE from ENCOS) ---
    int enable_fd = 1;
    if (::setsockopt(sock_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES,
                     &enable_fd, sizeof(enable_fd)) < 0) {
        std::perror("[w3_bridge] setsockopt(CAN_RAW_FD_FRAMES)");
        // Non-fatal: will fall back to standard CAN frames if FD not supported
        std::fprintf(stderr, "[w3_bridge] WARN: CAN FD not available for %s, "
                     "falling back to standard CAN\n", cfg_.iface.c_str());
    }

    // Disable loopback (we don't want to receive our own frames)
    int loopback = 0;
    ::setsockopt(sock_, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback));

    // Disable receive own messages
    int recv_own_msgs = 0;
    ::setsockopt(sock_, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &recv_own_msgs, sizeof(recv_own_msgs));

    // --- Bind to interface ---
    ifreq ifr{};
    std::strncpy(ifr.ifr_name, cfg_.iface.c_str(), IFNAMSIZ - 1);
    if (::ioctl(sock_, SIOCGIFINDEX, &ifr) < 0) {
        std::fprintf(stderr, "[w3_bridge] SIOCGIFINDEX %s: %s\n",
                     cfg_.iface.c_str(), std::strerror(errno));
        return false;
    }

    sockaddr_can addr{};
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (::bind(sock_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        std::fprintf(stderr, "[w3_bridge] bind(%s): %s\n",
                     cfg_.iface.c_str(), std::strerror(errno));
        return false;
    }

    // --- Set receive timeout (500ms for clean shutdown) ---
    timeval tv{};
    tv.tv_sec  = 0;
    tv.tv_usec = 500000;  // 500ms
    ::setsockopt(sock_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    // --- Initialize per-motor state ---
    profiles_.clear();
    commands_.clear();
    states_.clear();
    can_id_to_slot_.clear();

    for (uint8_t i = 0; i < cfg_.motor_count && i < cfg_.profiles.size(); ++i) {
        profiles_.push_back(cfg_.profiles[i]);
        commands_.push_back(MotorCommand{});
        MotorState s{};
        s.can_id  = cfg_.profiles[i].can_id;
        s.online  = false;
        s.enabled = false;
        states_.push_back(s);

        // Register the motor's primary CAN ID for TX (command)
        // 达妙电机反馈 CAN ID = motor_id + 0x10
        //   motor_id=1 → 命令ID=0x01, 反馈ID=0x11
        //   motor_id=2 → 命令ID=0x02, 反馈ID=0x12
        can_id_to_slot_[cfg_.profiles[i].can_id] = i;           // TX command
        can_id_to_slot_[cfg_.profiles[i].can_id + 0x10] = i;    // RX feedback
    }

    std::printf("[w3_bridge] Channel %s opened: %u motors, %u bps/%u bps FD, %.0f Hz/motor\n",
                cfg_.iface.c_str(), cfg_.motor_count,
                cfg_.baudrate, cfg_.data_baudrate, cfg_.control_rate_hz);
    return true;
}

bool CanChannel::start() {
    if (running_.exchange(true)) return true;  // already running

    tx_thread_ = std::thread([this] { txLoop(); });
    rx_thread_ = std::thread([this] { rxLoop(); });

    return true;
}

void CanChannel::stop() {
    if (!running_.exchange(false)) return;

    // Wake up blocking read for clean shutdown
    if (sock_ >= 0) ::shutdown(sock_, SHUT_RDWR);

    if (tx_thread_.joinable()) tx_thread_.join();
    if (rx_thread_.joinable()) rx_thread_.join();
}

// ── Command interface ──────────────────────────────────────────

void CanChannel::setCommand(uint8_t motor_index, const MotorCommand& cmd) {
    if (!validIndex(motor_index)) return;
    std::lock_guard<std::mutex> lk(cmd_mutex_);
    MotorCommand clamped = cmd;
    protocol::apply_limits(clamped, profiles_[motor_index].limits);
    commands_[motor_index] = clamped;
}

void CanChannel::enableMotor(uint8_t motor_index) {
    if (!validIndex(motor_index)) return;

    // Per official damiao.cpp: enable = send 0xFC command frame.
    // CAN ID = can_id + MIT_MODE(0x000) = can_id
    // Data = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC}
    // Send 5 times (official code repeats for reliability).
    uint16_t can_id;
    {
        std::lock_guard<std::mutex> lk(cmd_mutex_);
        commands_[motor_index] = MotorCommand{
            MotorMode::RUN, 0.0f, 0.0f, 0.0f, 0.5f, 0.0f
        };
        can_id = profiles_[motor_index].can_id;
    }
    uint8_t payload[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
    for (int j = 0; j < 5; ++j) {
        sendFrame(can_id, payload);
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
}

void CanChannel::disableMotor(uint8_t motor_index) {
    if (!validIndex(motor_index)) return;

    // Per official damiao.cpp: disable = send 0xFD command frame.
    // Data = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD}
    uint16_t can_id;
    {
        std::lock_guard<std::mutex> lk(cmd_mutex_);
        commands_[motor_index] = MotorCommand{
            MotorMode::DISABLE, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f
        };
        can_id = profiles_[motor_index].can_id;
    }
    uint8_t payload[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
    for (int j = 0; j < 5; ++j) {
        sendFrame(can_id, payload);
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
}

void CanChannel::sendSetZero(uint8_t motor_index) {
    if (!validIndex(motor_index)) return;

    // Per official damiao.cpp: set zero = send 0xFE command frame.
    // Data = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE}
    uint16_t can_id;
    {
        std::lock_guard<std::mutex> lk(cmd_mutex_);
        can_id = profiles_[motor_index].can_id;
    }
    uint8_t payload[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};
    sendFrame(can_id, payload);
}

// ── State access ───────────────────────────────────────────────

MotorState CanChannel::getState(uint8_t motor_index) const {
    if (!validIndex(motor_index)) return {};

    // enabled from feedback D[0] high nibble: ERR=1→使能, other→失能/故障
    bool enabled = false;
    {
        std::lock_guard<std::mutex> lk(state_mutex_);
        enabled = states_[motor_index].enabled;
    }

    std::lock_guard<std::mutex> lk(state_mutex_);
    MotorState s = states_[motor_index];
    s.enabled = enabled;
    return s;
}

bool CanChannel::isOnline(uint8_t motor_index) const {
    if (!validIndex(motor_index)) return false;
    std::lock_guard<std::mutex> lk(state_mutex_);
    return states_[motor_index].online;
}

bool CanChannel::isEnabled(uint8_t motor_index) const {
    if (!validIndex(motor_index)) return false;
    std::lock_guard<std::mutex> lk(cmd_mutex_);
    return commands_[motor_index].mode != MotorMode::DISABLE;
}

const MotorProfile* CanChannel::getProfile(uint8_t motor_index) const {
    if (!validIndex(motor_index)) return nullptr;
    // profiles_ doesn't change after open() → no lock needed
    return &profiles_[motor_index];
}

// ── CAN FD Frame I/O ──────────────────────────────────────────
//
// Motor is configured for CAN FD 1M+5M. We use canfd_frame with
// CANFD_BRS flag to get 5Mbps data phase.

void CanChannel::sendFrame(uint16_t can_id, const uint8_t data[8]) {
    canfd_frame frame{};
    frame.can_id = can_id & CAN_SFF_MASK;  // standard 11-bit ID
    frame.flags  = CANFD_BRS;              // Bit Rate Switch → 5M data phase
    frame.len    = kCanFrameLen;           // always 8 bytes for MIT mode
    std::memcpy(frame.data, data, kCanFrameLen);

    const ssize_t n = ::write(sock_, &frame, sizeof(frame));
    if (n == static_cast<ssize_t>(sizeof(frame))) {
        tx_frames_.fetch_add(1, std::memory_order_relaxed);
        tx_error_count_ = 0;
    } else {
        ++tx_error_count_;
        if (tx_error_count_ % 1000 == 1) {
            std::fprintf(stderr, "[w3_bridge] TX write error on %s "
                         "(id=0x%03X, rc=%zd, errno=%d): %s\n",
                         cfg_.iface.c_str(), can_id, n, errno,
                         n < 0 ? std::strerror(errno) : "short write");
        }
    }
}

// ── TX Thread ──────────────────────────────────────────────────
//
// Round-robin scheduling: sends one CAN FD frame per motor per iteration.
// Period = 1 / (motor_count × control_rate_hz)
// For 8 motors @ 200Hz: period = 1 / 1600 = 625 microseconds
//
// This ensures each motor gets a command frame exactly at 200Hz,
// while the CAN bus sees 1600 frames/second (well within CAN FD 5M budget).

void CanChannel::txLoop() {
    using clock = std::chrono::steady_clock;

    const uint8_t n_motors = cfg_.motor_count;
    if (n_motors == 0) return;

    const double period_s = cfg_.control_rate_hz > 0.0
        ? 1.0 / (static_cast<double>(n_motors) * cfg_.control_rate_hz)
        : 0.01;
    const auto period = std::chrono::duration_cast<clock::duration>(
        std::chrono::duration<double>(period_s));

    auto next = clock::now();
    uint8_t slot = 0;

    while (running_.load(std::memory_order_acquire)) {
        // Send one frame for the current motor
        MotorCommand cmd;
        MotorProfile prof;
        {
            std::lock_guard<std::mutex> lk(cmd_mutex_);
            cmd  = commands_[slot];
            prof = profiles_[slot];
        }

        // DISABLE mode: don't send any frame. The motor's internal
        // watchdog (typically 500ms) will naturally stop it.
        // Sending an all-zero MIT frame would be misinterpreted as
        // a valid zero-torque command.
        if (prof.active && cmd.mode != MotorMode::DISABLE) {
            uint8_t payload[kCanFrameLen];
            protocol::encode_mit_command(cmd, prof, payload);
            sendFrame(prof.can_id, payload);
        }

        // Move to next motor
        slot = (slot + 1) % n_motors;
        if (slot == 0) {
            // Completed one full round — refresh online flags
            refreshOnlineFlags();
        }

        // Sleep until next slot time
        next += period;
        auto now = clock::now();
        if (next < now) {
            // We fell behind — resync to avoid burst
            next = now + period;
        }
        std::this_thread::sleep_until(next);
    }
}

// ── RX Thread ──────────────────────────────────────────────────
//
// Blocking read on the CAN FD socket. Each received frame is dispatched
// to the correct motor slot via can_id_to_slot_ map and decoded.

void CanChannel::rxLoop() {
    size_t unknown_id_log_count = 0;  // throttle unknown-ID logging

    while (running_.load(std::memory_order_acquire)) {
        canfd_frame frame{};
        const ssize_t n = ::read(sock_, &frame, sizeof(frame));
        if (n < 0) {
            if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) continue;
            break;  // socket closed or fatal error
        }
        if (n < static_cast<ssize_t>(sizeof(canfd_frame))) continue;

        const auto slot = resolve_feedback_slot(
            frame.can_id, frame.data, frame.len, can_id_to_slot_);
        const uint32_t can_id = frame.can_id & CAN_EFF_MASK;
        if (!slot) {
            if (unknown_id_log_count < 20) {
                std::fprintf(stderr,
                    "[w3_bridge] %s RX: unknown CAN ID 0x%03X (len=%u) — "
                    "not in motor slot map.\n",
                    cfg_.iface.c_str(), can_id, frame.len);
                ++unknown_id_log_count;
            }
            continue;
        }

        const MotorProfile& prof = profiles_[*slot];

        std::lock_guard<std::mutex> lk(state_mutex_);
        bool ok = protocol::decode_feedback(frame.data, prof, states_[*slot]);
        if (ok) {
            states_[*slot].can_id = can_id;
            rx_frames_.fetch_add(1, std::memory_order_relaxed);
        }
    }
}

// ── Online detection ───────────────────────────────────────────

void CanChannel::refreshOnlineFlags() {
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
