#include "c2/connection_monitor.hpp"

#include "c2/protocol_validation.hpp"

#include <stdexcept>

namespace c2 {
ConnectionMonitor::ConnectionMonitor(const ConnectionMonitorConfig config) : config_(config) {
    if (config_.heartbeat_timeout_us == 0)
        throw std::invalid_argument("heartbeat timeout must be non-zero");
}

HeartbeatUpdateResult ConnectionMonitor::observe(
    const Heartbeat& heartbeat, const std::uint64_t received_at_us) {
    if (received_at_us == 0 || !validate(heartbeat).valid())
        return HeartbeatUpdateResult::invalid;

    std::lock_guard lock(mutex_);
    auto* destination = entry(heartbeat.header.source_id);
    if (destination == nullptr) return HeartbeatUpdateResult::invalid;
    if (*destination) {
        auto& current = **destination;
        if (received_at_us < current.received_at_us)
            return HeartbeatUpdateResult::out_of_order;
        if (!disconnected(current, received_at_us)) {
            if (heartbeat.header.sequence == current.heartbeat.header.sequence &&
                heartbeat.timestamp_us == current.heartbeat.timestamp_us)
                return HeartbeatUpdateResult::duplicate;
            if (heartbeat.timestamp_us <= current.heartbeat.timestamp_us)
                return HeartbeatUpdateResult::out_of_order;
            const auto synchronized = !current.pose_resynchronization_required;
            current = {heartbeat, received_at_us, !synchronized};
            return HeartbeatUpdateResult::stored;
        }
    }
    *destination = Entry{heartbeat, received_at_us, true};
    return HeartbeatUpdateResult::stored;
}

ConnectionState ConnectionMonitor::state(
    const ComponentId source, const std::uint64_t now_us) {
    std::lock_guard lock(mutex_);
    auto* selected = entry(source);
    if (selected == nullptr) return ConnectionState::unsupported_source;
    if (!*selected) return ConnectionState::never_seen;
    if (disconnected(**selected, now_us)) {
        (**selected).pose_resynchronization_required = true;
        return ConnectionState::disconnected;
    }
    return ConnectionState::connected;
}

bool ConnectionMonitor::pose_resynchronization_required(const ComponentId source) const {
    std::lock_guard lock(mutex_);
    const auto* selected = entry(source);
    return selected != nullptr && *selected && (**selected).pose_resynchronization_required;
}

bool ConnectionMonitor::mark_pose_synchronized(
    const ComponentId source, const std::uint64_t now_us) {
    std::lock_guard lock(mutex_);
    auto* selected = entry(source);
    if (selected == nullptr || !*selected || disconnected(**selected, now_us)) return false;
    (**selected).pose_resynchronization_required = false;
    return true;
}

bool ConnectionMonitor::disconnected(
    const Entry& entry_value, const std::uint64_t now_us) const noexcept {
    return now_us > entry_value.received_at_us &&
           now_us - entry_value.received_at_us > config_.heartbeat_timeout_us;
}

std::optional<ConnectionMonitor::Entry>* ConnectionMonitor::entry(
    const ComponentId source) noexcept {
    if (source == ComponentId::observation_asset) return &observation_;
    if (source == ComponentId::effector_asset) return &effector_;
    return nullptr;
}

const std::optional<ConnectionMonitor::Entry>* ConnectionMonitor::entry(
    const ComponentId source) const noexcept {
    if (source == ComponentId::observation_asset) return &observation_;
    if (source == ComponentId::effector_asset) return &effector_;
    return nullptr;
}
}  // namespace c2
