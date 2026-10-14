#include "c2/state_store.hpp"

#include "c2/protocol_validation.hpp"

#include <stdexcept>
#include <algorithm>

namespace c2 {
StateStore::StateStore(const StateStoreConfig config) : config_(config) {
    if (config_.target_validity_us == 0)
        throw std::invalid_argument("target validity must be non-zero");
    if (config_.maximum_targets == 0)
        throw std::invalid_argument("maximum target count must be non-zero");
}

StoreUpdateResult StateStore::update(const AssetPose& pose) {
    if (!validate(pose).valid()) return StoreUpdateResult::invalid;

    std::lock_guard lock(mutex_);
    auto& current = pose.header.source_id == ComponentId::observation_asset
                        ? observation_pose_
                        : effector_pose_;
    if (current) {
        if (pose.header.sequence == current->header.sequence &&
            pose.header.timestamp_us == current->header.timestamp_us)
            return StoreUpdateResult::duplicate;
        if (pose.header.timestamp_us <= current->header.timestamp_us)
            return StoreUpdateResult::out_of_order;
    }
    current = pose;
    return StoreUpdateResult::stored;
}

StoreUpdateResult StateStore::update(
    const TargetCoordinate& target_value, const std::uint64_t now_us) {
    if (!validate(target_value).valid()) return StoreUpdateResult::invalid;
    if (target_expired(target_value, now_us)) return StoreUpdateResult::expired;

    std::lock_guard lock(mutex_);
    const auto existing = targets_.find(target_value.detection_id);
    if (existing != targets_.end()) {
        const auto& current = existing->second;
        if (target_value.header.sequence == current.header.sequence &&
            target_value.measurement_time_us == current.measurement_time_us)
            return StoreUpdateResult::duplicate;
        if (target_value.measurement_time_us <= current.measurement_time_us)
            return StoreUpdateResult::out_of_order;
        existing->second = target_value;
        return StoreUpdateResult::stored;
    }

    prune_expired_locked(now_us);
    if (targets_.size() >= config_.maximum_targets)
        return StoreUpdateResult::capacity_exceeded;
    targets_.emplace(target_value.detection_id, target_value);
    return StoreUpdateResult::stored;
}

std::optional<AssetPose> StateStore::asset_pose(const ComponentId source) const {
    std::lock_guard lock(mutex_);
    if (source == ComponentId::observation_asset) return observation_pose_;
    if (source == ComponentId::effector_asset) return effector_pose_;
    return std::nullopt;
}

std::optional<TargetCoordinate> StateStore::target(
    const std::uint32_t detection_id, const std::uint64_t now_us) const {
    std::lock_guard lock(mutex_);
    const auto found = targets_.find(detection_id);
    if (found == targets_.end() || target_expired(found->second, now_us))
        return std::nullopt;
    return found->second;
}

std::vector<TargetCoordinate> StateStore::targets(const std::uint64_t now_us) const {
    std::lock_guard lock(mutex_);
    std::vector<TargetCoordinate> current;
    current.reserve(targets_.size());
    for (const auto& [id, target] : targets_) {
        (void)id;
        if (!target_expired(target, now_us)) current.push_back(target);
    }
    std::sort(current.begin(), current.end(), [](const auto& left, const auto& right) {
        return left.detection_id < right.detection_id;
    });
    return current;
}

std::size_t StateStore::target_count(const std::uint64_t now_us) const {
    std::lock_guard lock(mutex_);
    std::size_t count{};
    for (const auto& [id, candidate] : targets_) {
        (void)id;
        if (!target_expired(candidate, now_us)) ++count;
    }
    return count;
}

std::size_t StateStore::prune_expired(const std::uint64_t now_us) {
    std::lock_guard lock(mutex_);
    return prune_expired_locked(now_us);
}

bool StateStore::target_expired(
    const TargetCoordinate& target_value, const std::uint64_t now_us) const noexcept {
    return now_us > target_value.measurement_time_us &&
           now_us - target_value.measurement_time_us > config_.target_validity_us;
}

std::size_t StateStore::prune_expired_locked(const std::uint64_t now_us) {
    std::size_t removed{};
    for (auto current = targets_.begin(); current != targets_.end();) {
        if (target_expired(current->second, now_us)) {
            current = targets_.erase(current);
            ++removed;
        } else {
            ++current;
        }
    }
    return removed;
}
}  // namespace c2
