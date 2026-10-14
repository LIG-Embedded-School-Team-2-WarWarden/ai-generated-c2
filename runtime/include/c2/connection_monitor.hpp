#pragma once

#include "c2/protocol.hpp"

#include <cstdint>
#include <mutex>
#include <optional>

namespace c2 {
struct ConnectionMonitorConfig {
    std::uint64_t heartbeat_timeout_us{};
};

enum class ConnectionState { never_seen, connected, disconnected, unsupported_source };
enum class HeartbeatUpdateResult { stored, invalid, duplicate, out_of_order };

class ConnectionMonitor final {
public:
    explicit ConnectionMonitor(ConnectionMonitorConfig config);

    [[nodiscard]] HeartbeatUpdateResult observe(
        const Heartbeat& heartbeat, std::uint64_t received_at_us);
    [[nodiscard]] ConnectionState state(ComponentId source, std::uint64_t now_us);
    [[nodiscard]] bool pose_resynchronization_required(ComponentId source) const;
    [[nodiscard]] bool mark_pose_synchronized(ComponentId source, std::uint64_t now_us);

private:
    struct Entry {
        Heartbeat heartbeat;
        std::uint64_t received_at_us{};
        bool pose_resynchronization_required{true};
    };

    [[nodiscard]] bool disconnected(const Entry& entry, std::uint64_t now_us) const noexcept;
    [[nodiscard]] std::optional<Entry>* entry(ComponentId source) noexcept;
    [[nodiscard]] const std::optional<Entry>* entry(ComponentId source) const noexcept;

    ConnectionMonitorConfig config_;
    mutable std::mutex mutex_;
    std::optional<Entry> observation_;
    std::optional<Entry> effector_;
};
}  // namespace c2
