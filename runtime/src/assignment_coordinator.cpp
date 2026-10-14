#include "c2/assignment_coordinator.hpp"
namespace c2 {
AssignmentDecision AssignmentCoordinator::assign(
    const std::uint64_t track_id, const std::uint64_t now_us,
    const std::optional<std::uint64_t> requested_asset_id) {
    const auto track = tracks_.track(track_id, now_us);
    if (!track) return {AssignmentResult::invalid_track, std::nullopt};
    auto candidates = effector_candidates(now_us);
    invalidate_unsafe_assignment(*track, candidates, now_us);
    auto decision = assignments_.assign(*track, candidates, requested_asset_id);
    if (decision.result == AssignmentResult::assigned && decision.assignment) {
        const auto asset = registry_.asset(decision.assignment->effector_asset_id, now_us);
        if (asset) track_updates_.publish(*track, *asset, now_us);
    }
    return decision;
}

std::vector<EffectorCandidate> AssignmentCoordinator::effector_candidates(
    const std::uint64_t now_us) {
    std::vector<EffectorCandidate> candidates;
    for (auto& asset : registry_.assets(AssetRole::effector, now_us)) {
        candidates.push_back({asset, asset.effector_status, asset.status_current,
                              false, false, 0, 0});
    }
    return candidates;
}

void AssignmentCoordinator::invalidate_unsafe_assignment(
    const TrackSnapshot& track,
    const std::vector<EffectorCandidate>& candidates,
    const std::uint64_t now_us) {
    const auto existing = assignments_.assignment(track.track_id);
    if (!existing || existing->state != AssignmentResult::assigned) return;
    std::vector<EffectorCandidate> assigned_candidate;
    for (const auto& candidate : candidates) {
        if (candidate.asset.asset_id == existing->effector_asset_id &&
            candidate.asset.session_id == existing->effector_session_id) {
            assigned_candidate.push_back(candidate);
            break;
        }
    }
    if (!select_effector_candidate(
            track, assigned_candidate, config_.required_capabilities,
            config_.weights))
        (void)assignments_.mark_unavailable(
            existing->effector_asset_id, existing->effector_session_id, now_us);
}

} // namespace c2
