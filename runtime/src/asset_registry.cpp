#include "c2/asset_registry.hpp"

#include "c2/protocol_validation.hpp"

#include <algorithm>
#include <limits>
#include <stdexcept>

namespace c2 {
namespace {
bool same_endpoint(const Endpoint& left, const Endpoint& right) noexcept {
    return left.address == right.address && left.port == right.port;
}

std::uint64_t deadline_after_ms(
    const std::uint64_t now_us, const std::uint64_t duration_ms) noexcept {
    if (duration_ms > (std::numeric_limits<std::uint64_t>::max() - now_us) / 1000U)
        return std::numeric_limits<std::uint64_t>::max();
    return now_us + duration_ms * 1000U;
}
}  // namespace

AssetRegistry::AssetRegistry(const AssetRegistryConfig config) : config_(config) {
    if (config_.maximum_assets == 0)
        throw std::invalid_argument("maximum asset count must be non-zero");
    if (config_.heartbeat_timeout_us == 0)
        throw std::invalid_argument("heartbeat timeout must be non-zero");
    if (config_.retired_retention_us == 0)
        throw std::invalid_argument("retired retention must be non-zero");
    if (config_.status_timeout_us == 0)
        throw std::invalid_argument("status timeout must be non-zero");
    if (config_.maximum_lease_duration_ms == 0)
        throw std::invalid_argument("maximum registration lease must be non-zero");
}

AssetRegistryResult AssetRegistry::register_asset(
    const AssetRegistration& registration,
    const Endpoint& source,
    const std::uint64_t received_at_us) {
    if (!validate(registration).valid() ||
        registration.lease_duration_ms > config_.maximum_lease_duration_ms ||
        source.address.empty() || source.port == 0 ||
        received_at_us == 0)
        return AssetRegistryResult::invalid;

    std::lock_guard lock(mutex_);
    const auto found = entries_.find(registration.header.asset_id);
    if (found == entries_.end()) {
        if (entries_.size() >= config_.maximum_assets)
            return AssetRegistryResult::capacity_exceeded;
        Entry entry;
        entry.registration = registration;
        entry.source_endpoint = source;
        entry.command_endpoint = {source.address,
                                  static_cast<std::uint16_t>(registration.command_port)};
        entry.registered_at_us = received_at_us;
        entry.lease_expires_at_us = deadline_after_ms(
            received_at_us, registration.lease_duration_ms);
        entries_.emplace(registration.header.asset_id, std::move(entry));
        return AssetRegistryResult::registered;
    }

    auto& entry = found->second;
    if (entry.registration.role != registration.role)
        return AssetRegistryResult::role_mismatch;
    if (entry.active &&
        entry.registration.header.session_id == registration.header.session_id) {
        if (!same_endpoint(entry.source_endpoint, source))
            return AssetRegistryResult::endpoint_mismatch;
        entry.registration = registration;
        entry.command_endpoint.port = static_cast<std::uint16_t>(registration.command_port);
        entry.lease_expires_at_us = deadline_after_ms(
            received_at_us, registration.lease_duration_ms);
        return AssetRegistryResult::refreshed;
    }

    entry = Entry{};
    entry.registration = registration;
    entry.source_endpoint = source;
    entry.command_endpoint = {source.address,
                              static_cast<std::uint16_t>(registration.command_port)};
    entry.registered_at_us = received_at_us;
    entry.lease_expires_at_us = deadline_after_ms(
        received_at_us, registration.lease_duration_ms);
    return AssetRegistryResult::session_replaced;
}

AssetRegistryResult AssetRegistry::validate_message_locked(
    const MessageHeader& header, const Endpoint& source, const Entry*& entry) const {
    const auto found = entries_.find(header.asset_id);
    if (found == entries_.end() || !found->second.active)
        return AssetRegistryResult::not_registered;
    entry = &found->second;
    if (entry->registration.header.session_id != header.session_id)
        return AssetRegistryResult::session_mismatch;
    if (!same_endpoint(entry->source_endpoint, source))
        return AssetRegistryResult::endpoint_mismatch;
    if (entry->registration.header.source_id != header.source_id)
        return AssetRegistryResult::role_mismatch;
    return AssetRegistryResult::stored;
}

AssetRegistryResult AssetRegistry::unregister_asset(
    const AssetUnregister& unregister_message,
    const Endpoint& source,
    const std::uint64_t received_at_us) {
    if (!validate(unregister_message).valid() || received_at_us == 0)
        return AssetRegistryResult::invalid;
    std::lock_guard lock(mutex_);
    const Entry* validated{};
    const auto result = validate_message_locked(
        unregister_message.header, source, validated);
    if (result != AssetRegistryResult::stored) return result;
    auto& entry = entries_.at(unregister_message.header.asset_id);
    entry.active = false;
    entry.terminal_state = AssetConnectionState::unregistered;
    entry.retired_at_us = received_at_us;
    entry.pose.reset();
    entry.effector_status.reset();
    return AssetRegistryResult::unregistered;
}

AssetRegistryResult AssetRegistry::observe_heartbeat(
    const Heartbeat& heartbeat_value,
    const Endpoint& source,
    const std::uint64_t received_at_us) {
    if (!validate(heartbeat_value).valid() || received_at_us == 0)
        return AssetRegistryResult::invalid;
    std::lock_guard lock(mutex_);
    const Entry* validated{};
    const auto result = validate_message_locked(
        heartbeat_value.header, source, validated);
    if (result != AssetRegistryResult::stored) return result;
    auto& entry = entries_.at(heartbeat_value.header.asset_id);
    if (entry.heartbeat) {
        if (heartbeat_value.header.sequence == entry.heartbeat->header.sequence &&
            heartbeat_value.timestamp_us == entry.heartbeat->timestamp_us)
            return AssetRegistryResult::duplicate;
        if (heartbeat_value.timestamp_us <= entry.heartbeat->timestamp_us)
            return AssetRegistryResult::stale;
    }
    entry.heartbeat = heartbeat_value;
    entry.last_heartbeat_received_at_us = received_at_us;
    return AssetRegistryResult::stored;
}

AssetRegistryResult AssetRegistry::update_pose(
    const AssetPose& pose_value,
    const Endpoint& source,
    const std::uint64_t received_at_us) {
    if (!validate(pose_value).valid() || received_at_us == 0)
        return AssetRegistryResult::invalid;
    std::lock_guard lock(mutex_);
    const Entry* validated{};
    const auto result = validate_message_locked(pose_value.header, source, validated);
    if (result != AssetRegistryResult::stored) return result;
    auto& entry = entries_.at(pose_value.header.asset_id);
    if (entry.pose) {
        if (pose_value.header.sequence == entry.pose->header.sequence &&
            pose_value.header.timestamp_us == entry.pose->header.timestamp_us)
            return AssetRegistryResult::duplicate;
        if (pose_value.header.timestamp_us <= entry.pose->header.timestamp_us)
            return AssetRegistryResult::stale;
    }
    entry.pose = pose_value;
    return AssetRegistryResult::stored;
}

AssetRegistryResult AssetRegistry::authenticate(
    const MessageHeader& header,
    const Endpoint& source,
    const std::uint64_t received_at_us) {
    if (received_at_us == 0) return AssetRegistryResult::invalid;
    std::lock_guard lock(mutex_);
    const Entry* entry{};
    const auto result = validate_message_locked(header, source, entry);
    if (result != AssetRegistryResult::stored) return result;
    if (received_at_us >= entry->lease_expires_at_us)
        return AssetRegistryResult::not_registered;
    return AssetRegistryResult::stored;
}

AssetRegistryResult AssetRegistry::update_effector_status(
    const EffectorStatus& status,
    const Endpoint& source,
    const std::uint64_t received_at_us) {
    if (!validate(status).valid() || received_at_us == 0)
        return AssetRegistryResult::invalid;
    std::lock_guard lock(mutex_);
    const Entry* validated{};
    const auto result = validate_message_locked(status.header, source, validated);
    if (result != AssetRegistryResult::stored) return result;
    auto& entry = entries_.at(status.header.asset_id);
    if (entry.registration.role != AssetRole::effector)
        return AssetRegistryResult::role_mismatch;
    if (entry.effector_status) {
        if (status.header.sequence == entry.effector_status->header.sequence &&
            status.timestamp_us == entry.effector_status->timestamp_us)
            return AssetRegistryResult::duplicate;
        if (status.timestamp_us <= entry.effector_status->timestamp_us)
            return AssetRegistryResult::stale;
    }
    entry.effector_status = status;
    entry.status_received_at_us = received_at_us;
    return AssetRegistryResult::stored;
}

AssetConnectionState AssetRegistry::state_locked(
    const Entry& entry, const std::uint64_t now_us) const noexcept {
    if (!entry.active) return entry.terminal_state;
    if (now_us >= entry.lease_expires_at_us)
        return AssetConnectionState::lease_expired;
    if (!entry.heartbeat) return AssetConnectionState::awaiting_heartbeat;
    if (now_us > entry.last_heartbeat_received_at_us &&
        now_us - entry.last_heartbeat_received_at_us > config_.heartbeat_timeout_us)
        return AssetConnectionState::disconnected;
    return AssetConnectionState::connected;
}

AssetSnapshot AssetRegistry::snapshot_locked(
    const Entry& entry, const std::uint64_t now_us) const {
    const auto& registration = entry.registration;
    const auto status_current = entry.effector_status &&
        (now_us <= entry.status_received_at_us ||
         now_us - entry.status_received_at_us <= config_.status_timeout_us);
    return {registration.header.asset_id,
            registration.header.session_id,
            registration.role,
            registration.capabilities,
            registration.software_version,
            registration.hardware_version,
            registration.turret_limits,
            registration.concurrent_tasks,
            entry.command_endpoint,
            entry.source_endpoint,
            entry.registered_at_us,
            entry.lease_expires_at_us,
            entry.last_heartbeat_received_at_us,
            state_locked(entry, now_us),
            entry.pose.has_value(),
            entry.pose,
            entry.effector_status,
            status_current};
}

std::optional<AssetSnapshot> AssetRegistry::asset(
    const std::uint64_t asset_id, const std::uint64_t now_us) const {
    std::lock_guard lock(mutex_);
    const auto found = entries_.find(asset_id);
    if (found == entries_.end()) return std::nullopt;
    return snapshot_locked(found->second, now_us);
}

std::optional<AssetConnectionState> AssetRegistry::connection_state(
    const std::uint64_t asset_id, const std::uint64_t now_us) const {
    std::lock_guard lock(mutex_);
    const auto found = entries_.find(asset_id);
    if (found == entries_.end()) return std::nullopt;
    return state_locked(found->second, now_us);
}

std::vector<AssetSnapshot> AssetRegistry::assets(
    const AssetRole role, const std::uint64_t now_us) {
    (void)expire(now_us);
    std::lock_guard lock(mutex_);
    std::vector<AssetSnapshot> result;
    for (const auto& [id, entry] : entries_) {
        (void)id;
        if (entry.active && entry.registration.role == role)
            result.push_back(snapshot_locked(entry, now_us));
    }
    return result;
}

std::vector<AssetSnapshot> AssetRegistry::assets(const std::uint64_t now_us) {
    (void)expire(now_us);
    std::lock_guard lock(mutex_);
    std::vector<AssetSnapshot> result;
    for (const auto& [id, entry] : entries_) {
        (void)id;
        if (entry.active) result.push_back(snapshot_locked(entry, now_us));
    }
    return result;
}

std::vector<AssetSnapshot> AssetRegistry::known_assets(
    const AssetRole role, const std::uint64_t now_us) {
    (void)prune(now_us);
    std::lock_guard lock(mutex_);
    std::vector<AssetSnapshot> result;
    for (const auto& [id, entry] : entries_) {
        (void)id;
        if (entry.registration.role == role)
            result.push_back(snapshot_locked(entry, now_us));
    }
    return result;
}

void AssetRegistry::expire_locked(const std::uint64_t now_us, std::size_t& count) {
    for (auto& [id, entry] : entries_) {
        (void)id;
        if (entry.active && now_us >= entry.lease_expires_at_us) {
            entry.active = false;
            entry.terminal_state = AssetConnectionState::lease_expired;
            entry.retired_at_us = now_us;
            entry.pose.reset();
            entry.effector_status.reset();
            ++count;
        }
    }
}

std::size_t AssetRegistry::expire(const std::uint64_t now_us) {
    std::lock_guard lock(mutex_);
    std::size_t count{};
    expire_locked(now_us, count);
    return count;
}

std::size_t AssetRegistry::prune(const std::uint64_t now_us) {
    std::lock_guard lock(mutex_);
    std::size_t expired{};
    expire_locked(now_us, expired);
    std::size_t removed{};
    for (auto iterator = entries_.begin(); iterator != entries_.end();) {
        const auto& entry = iterator->second;
        if (!entry.active && now_us > entry.retired_at_us &&
            now_us - entry.retired_at_us > config_.retired_retention_us) {
            iterator = entries_.erase(iterator);
            ++removed;
        } else {
            ++iterator;
        }
    }
    return removed;
}
}  // namespace c2
