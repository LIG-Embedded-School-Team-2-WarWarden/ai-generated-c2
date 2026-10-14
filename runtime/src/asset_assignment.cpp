#include "c2/asset_assignment.hpp"

#include "c2/pointing.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <variant>

namespace c2 {
namespace {
bool valid_weights(const AssignmentScoreWeights& weights) noexcept {
    const double values[]{weights.distance_weight, weights.rotation_weight,
                          weights.assignment_weight, weights.degraded_penalty,
                          weights.failure_weight};
    for (const auto value : values) {
        if (!std::isfinite(value) || value < 0) return false;
    }
    return true;
}

double angular_distance(const double from, const double to) noexcept {
    auto difference = std::fmod(std::abs(to - from), 360.0);
    if (difference > 180.0) difference = 360.0 - difference;
    return difference;
}

bool status_available(const EffectorCandidate& candidate) noexcept {
    if (!candidate.status || !candidate.status_current || candidate.faulted)
        return false;
    const auto& status = *candidate.status;
    return status.state != EffectorState::fault &&
           status.state != EffectorState::active &&
           !status.attack_active && status.error_code == 0;
}
}

std::vector<EffectorScore> rank_effector_candidates(
    const TrackSnapshot& track,
    const std::vector<EffectorCandidate>& candidates,
    const std::uint64_t required_capabilities,
    const AssignmentScoreWeights& weights) {
    if (!valid_weights(weights))
        throw std::invalid_argument("assignment score weights are invalid");
    std::vector<EffectorScore> result;
    for (const auto& candidate : candidates) {
        const auto& asset = candidate.asset;
        if (asset.role != AssetRole::effector ||
            asset.connection_state != AssetConnectionState::connected ||
            !asset.pose_synchronized || !asset.pose || !status_available(candidate) ||
            (asset.capabilities & required_capabilities) != required_capabilities ||
            (candidate.active_assignments != 0 && !asset.concurrent_tasks))
            continue;
        const PointingLimits limits{asset.turret_limits.minimum_pan_deg,
                                    asset.turret_limits.maximum_pan_deg,
                                    asset.turret_limits.minimum_tilt_deg,
                                    asset.turret_limits.maximum_tilt_deg};
        const auto pointing = calculate_effector_pointing(
            track.measurement, *asset.pose, limits);
        if (!std::holds_alternative<PointingSolution>(pointing)) continue;
        const auto& solution = std::get<PointingSolution>(pointing);
        const auto distance = std::hypot(
            static_cast<double>(solution.relative_x_m),
            static_cast<double>(solution.relative_y_m),
            static_cast<double>(solution.relative_z_m));
        const auto rotation = angular_distance(
            candidate.status->current_pan_deg, solution.pan_deg) +
            std::abs(static_cast<double>(candidate.status->current_tilt_deg) -
                     solution.tilt_deg);
        const auto total = distance * weights.distance_weight +
                           rotation * weights.rotation_weight +
                           candidate.active_assignments * weights.assignment_weight +
                           (candidate.degraded ? weights.degraded_penalty : 0.0) +
                           candidate.recent_failures * weights.failure_weight;
        result.push_back({asset.asset_id, asset.session_id, total, distance,
                          rotation, solution.pan_deg, solution.tilt_deg});
    }
    std::sort(result.begin(), result.end(), [](const auto& left, const auto& right) {
        if (left.total != right.total) return left.total < right.total;
        return left.asset_id < right.asset_id;
    });
    return result;
}

std::optional<EffectorScore> select_effector_candidate(
    const TrackSnapshot& track,
    const std::vector<EffectorCandidate>& candidates,
    const std::uint64_t required_capabilities,
    const AssignmentScoreWeights& weights) {
    auto ranked = rank_effector_candidates(
        track, candidates, required_capabilities, weights);
    if (ranked.empty()) return std::nullopt;
    return ranked.front();
}

AssetAssignmentService::AssetAssignmentService(AssetAssignmentConfig config)
    : config_(std::move(config)) {
    if (config_.required_capabilities == 0 || config_.maximum_assignments == 0)
        throw std::invalid_argument("assignment service configuration is invalid");
    (void)rank_effector_candidates(
        TrackSnapshot{}, {}, config_.required_capabilities, config_.weights);
}

std::size_t AssetAssignmentService::active_count_locked(
    const std::uint64_t asset_id) const {
    std::size_t count{};
    for (const auto& [track_id, assignment] : assignments_) {
        (void)track_id;
        if (assignment.effector_asset_id == asset_id &&
            (assignment.state == AssignmentResult::assigned ||
             assignment.state == AssignmentResult::operator_action_required))
            ++count;
    }
    return count;
}

AssignmentDecision AssetAssignmentService::assign(
    const TrackSnapshot& track,
    const std::vector<EffectorCandidate>& candidates,
    const std::optional<std::uint64_t> requested_asset_id) {
    if (track.track_id == 0) return {AssignmentResult::invalid_track, std::nullopt};
    std::lock_guard lock(mutex_);
    const auto existing = assignments_.find(track.track_id);
    if (existing != assignments_.end()) {
        if (existing->second.state == AssignmentResult::assigned)
            return {AssignmentResult::assigned, existing->second};
        if (existing->second.state == AssignmentResult::operator_action_required)
            return {AssignmentResult::operator_action_required, existing->second};
        if (existing->second.state == AssignmentResult::assignment_lost &&
            !config_.auto_reassignment_enabled && !requested_asset_id)
            return {AssignmentResult::assignment_lost, existing->second};
        if (existing->second.state == AssignmentResult::completed ||
            existing->second.state == AssignmentResult::failed)
            return {existing->second.state, existing->second};
    } else if (assignments_.size() >= config_.maximum_assignments) {
        return {AssignmentResult::capacity_exceeded, std::nullopt};
    }

    std::vector<EffectorCandidate> eligible_scope;
    bool relevant_candidate_exists{};
    for (auto candidate : candidates) {
        if (requested_asset_id && candidate.asset.asset_id != *requested_asset_id)
            continue;
        if (candidate.asset.role == AssetRole::effector &&
            (candidate.asset.capabilities & config_.required_capabilities) ==
                config_.required_capabilities)
            relevant_candidate_exists = true;
        candidate.active_assignments += active_count_locked(candidate.asset.asset_id);
        eligible_scope.push_back(std::move(candidate));
    }
    const auto selected = select_effector_candidate(
        track, eligible_scope, config_.required_capabilities, config_.weights);
    if (!selected) {
        const auto result = relevant_candidate_exists
            ? AssignmentResult::temporarily_unavailable
            : AssignmentResult::no_candidate;
        return {result, std::nullopt};
    }

    AssetAssignment assignment{track.track_id,
                               selected->asset_id,
                               selected->session_id,
                               AssignmentResult::assigned,
                               requested_asset_id.has_value(),
                               false,
                               selected->total,
                               track.received_at_us};
    assignments_.insert_or_assign(track.track_id, assignment);
    return {AssignmentResult::assigned, assignment};
}

std::optional<AssetAssignment> AssetAssignmentService::assignment(
    const std::uint64_t track_id) const {
    std::lock_guard lock(mutex_);
    const auto found = assignments_.find(track_id);
    if (found == assignments_.end()) return std::nullopt;
    return found->second;
}

std::vector<AssetAssignment> AssetAssignmentService::assignments() const {
    std::lock_guard lock(mutex_);
    std::vector<AssetAssignment> result;
    result.reserve(assignments_.size());
    for (const auto& [track_id, assignment] : assignments_) {
        (void)track_id;
        result.push_back(assignment);
    }
    return result;
}

bool AssetAssignmentService::mark_attack_started(const std::uint64_t track_id) {
    std::lock_guard lock(mutex_);
    const auto found = assignments_.find(track_id);
    if (found == assignments_.end() ||
        found->second.state != AssignmentResult::assigned)
        return false;
    found->second.attack_started = true;
    return true;
}

std::size_t AssetAssignmentService::mark_unavailable(
    const std::uint64_t asset_id, const std::uint64_t session_id,
    const std::uint64_t now_us) {
    std::lock_guard lock(mutex_);
    std::size_t changed{};
    for (auto& [track_id, assignment] : assignments_) {
        (void)track_id;
        if (assignment.effector_asset_id != asset_id ||
            assignment.effector_session_id != session_id ||
            assignment.state != AssignmentResult::assigned)
            continue;
        assignment.state = assignment.attack_started
            ? AssignmentResult::operator_action_required
            : AssignmentResult::assignment_lost;
        assignment.changed_at_us = now_us;
        ++changed;
    }
    return changed;
}

AssignmentResult AssetAssignmentService::unassign(const std::uint64_t track_id) {
    std::lock_guard lock(mutex_);
    const auto found = assignments_.find(track_id);
    if (found == assignments_.end()) return AssignmentResult::invalid_track;
    if (found->second.attack_started ||
        found->second.state == AssignmentResult::operator_action_required)
        return AssignmentResult::operator_action_required;
    found->second.state = AssignmentResult::completed;
    return AssignmentResult::completed;
}

AssignmentResult AssetAssignmentService::complete(
    const std::uint64_t track_id, const bool succeeded) {
    std::lock_guard lock(mutex_);
    const auto found = assignments_.find(track_id);
    if (found == assignments_.end()) return AssignmentResult::invalid_track;
    found->second.state = succeeded ? AssignmentResult::completed
                                    : AssignmentResult::failed;
    return found->second.state;
}
}  // namespace c2
