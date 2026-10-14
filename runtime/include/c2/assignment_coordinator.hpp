#pragma once
#include "c2/asset_assignment.hpp"
#include "c2/track_update_publisher.hpp"

namespace c2 {
class AssignmentCoordinator final {
public:
    AssignmentCoordinator(AssetRegistry& registry, TrackStore& tracks,
        AssetAssignmentService& assignments, TrackUpdatePublisher& track_updates,
        AssetAssignmentConfig config)
        : registry_(registry), tracks_(tracks), assignments_(assignments),
          track_updates_(track_updates), config_(std::move(config)) {}
    [[nodiscard]] AssignmentDecision assign(std::uint64_t track_id, std::uint64_t now_us,
        std::optional<std::uint64_t> requested_asset_id = std::nullopt);
private:
    [[nodiscard]] std::vector<EffectorCandidate> effector_candidates(std::uint64_t now_us);
    void invalidate_unsafe_assignment(const TrackSnapshot& track,
        const std::vector<EffectorCandidate>& candidates, std::uint64_t now_us);
    AssetRegistry& registry_;
    TrackStore& tracks_;
    AssetAssignmentService& assignments_;
    TrackUpdatePublisher& track_updates_;
    AssetAssignmentConfig config_;
};
} // namespace c2
