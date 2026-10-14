#pragma once

#include "c2/protocol.hpp"

#include <cstddef>
#include <cstdint>
#include <mutex>
#include <optional>
#include <unordered_map>
#include <vector>

namespace c2 {
struct StateStoreConfig {
    std::uint64_t target_validity_us{};
    std::size_t maximum_targets{};
};

enum class StoreUpdateResult {
    stored,
    invalid,
    duplicate,
    out_of_order,
    expired,
    capacity_exceeded,
};

class StateStore final {
public:
    explicit StateStore(StateStoreConfig config);

    [[nodiscard]] StoreUpdateResult update(const AssetPose& pose);
    [[nodiscard]] StoreUpdateResult update(
        const TargetCoordinate& target, std::uint64_t now_us);

    [[nodiscard]] std::optional<AssetPose> asset_pose(ComponentId source) const;
    [[nodiscard]] std::optional<TargetCoordinate> target(
        std::uint32_t detection_id, std::uint64_t now_us) const;
    [[nodiscard]] std::vector<TargetCoordinate> targets(std::uint64_t now_us) const;
    [[nodiscard]] std::size_t target_count(std::uint64_t now_us) const;
    std::size_t prune_expired(std::uint64_t now_us);

private:
    [[nodiscard]] bool target_expired(
        const TargetCoordinate& target, std::uint64_t now_us) const noexcept;
    std::size_t prune_expired_locked(std::uint64_t now_us);

    StateStoreConfig config_;
    mutable std::mutex mutex_;
    std::optional<AssetPose> observation_pose_;
    std::optional<AssetPose> effector_pose_;
    std::unordered_map<std::uint32_t, TargetCoordinate> targets_;
};
}  // namespace c2
