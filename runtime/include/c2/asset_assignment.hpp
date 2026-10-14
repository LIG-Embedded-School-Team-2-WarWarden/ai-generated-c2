#pragma once

#include "c2/asset_registry.hpp"
#include "c2/track_store.hpp"

#include <cstdint>
#include <map>
#include <mutex>
#include <optional>
#include <vector>

namespace c2 {
struct AssignmentScoreWeights {
    double distance_weight{1.0};
    double rotation_weight{1.0};
    double assignment_weight{100.0};
    double degraded_penalty{500.0};
    double failure_weight{25.0};
};

struct EffectorCandidate {
    AssetSnapshot asset;
    std::optional<EffectorStatus> status;
    bool status_current{};
    bool faulted{};
    bool degraded{};
    std::uint32_t recent_failures{};
    std::size_t active_assignments{};
};

struct EffectorScore {
    std::uint64_t asset_id{};
    std::uint64_t session_id{};
    double total{};
    double distance_m{};
    double rotation_deg{};
    float target_pan_deg{};
    float target_tilt_deg{};
};

[[nodiscard]] std::vector<EffectorScore> rank_effector_candidates(
    const TrackSnapshot& track,
    const std::vector<EffectorCandidate>& candidates,
    std::uint64_t required_capabilities,
    const AssignmentScoreWeights& weights);

[[nodiscard]] std::optional<EffectorScore> select_effector_candidate(
    const TrackSnapshot& track,
    const std::vector<EffectorCandidate>& candidates,
    std::uint64_t required_capabilities,
    const AssignmentScoreWeights& weights);

enum class AssignmentResult {
    assigned,
    no_candidate,
    temporarily_unavailable,
    assignment_lost,
    completed,
    failed,
    operator_action_required,
    capacity_exceeded,
    invalid_track,
};

struct AssetAssignmentConfig {
    AssignmentScoreWeights weights;
    std::uint64_t required_capabilities{
        capability::effector_point | capability::effector_attack};
    std::size_t maximum_assignments{256};
    bool auto_reassignment_enabled{true};
};

struct AssetAssignment {
    std::uint64_t track_id{};
    std::uint64_t effector_asset_id{};
    std::uint64_t effector_session_id{};
    AssignmentResult state{AssignmentResult::assigned};
    bool manually_selected{};
    bool attack_started{};
    double score{};
    std::uint64_t changed_at_us{};
};

struct AssignmentDecision {
    AssignmentResult result{AssignmentResult::no_candidate};
    std::optional<AssetAssignment> assignment;
};

class AssetAssignmentService final {
public:
    explicit AssetAssignmentService(AssetAssignmentConfig config);

    [[nodiscard]] AssignmentDecision assign(
        const TrackSnapshot& track,
        const std::vector<EffectorCandidate>& candidates,
        std::optional<std::uint64_t> requested_asset_id = std::nullopt);
    [[nodiscard]] std::optional<AssetAssignment> assignment(
        std::uint64_t track_id) const;
    [[nodiscard]] std::vector<AssetAssignment> assignments() const;
    [[nodiscard]] bool mark_attack_started(std::uint64_t track_id);
    [[nodiscard]] std::size_t mark_unavailable(
        std::uint64_t asset_id, std::uint64_t session_id,
        std::uint64_t now_us);
    [[nodiscard]] AssignmentResult unassign(std::uint64_t track_id);
    [[nodiscard]] AssignmentResult complete(
        std::uint64_t track_id, bool succeeded);

private:
    [[nodiscard]] std::size_t active_count_locked(
        std::uint64_t asset_id) const;

    AssetAssignmentConfig config_;
    mutable std::mutex mutex_;
    std::map<std::uint64_t, AssetAssignment> assignments_;
};
}  // namespace c2
