#pragma once

#include "c2/pointing.hpp"
#include "c2/protocol.hpp"
#include "c2/command_identity.hpp"
#include "c2/state_store.hpp"

#include <cstdint>
#include <mutex>
#include <variant>

namespace c2 {
struct EffectorCommandConfig {
    PointingLimits limits;
    std::uint64_t command_validity_us{};
    std::uint32_t first_command_id{1};
    std::uint32_t first_sequence{1};
};

enum class EffectorCommandError {
    target_unavailable,
    effector_pose_unavailable,
    invalid_target,
    invalid_effector_pose,
    coincident_target,
    outside_turret_limits,
    invalid_time,
    deadline_overflow,
};

using EffectorCommandResult =
    std::variant<EffectorTurretCommand, EffectorCommandError>;

class EffectorCommandService final {
public:
    EffectorCommandService(StateStore& state, EffectorCommandConfig config);

    [[nodiscard]] EffectorCommandResult create_for_target(
        std::uint64_t target_id, std::uint64_t now_us);
    [[nodiscard]] EffectorCommandResult create_for_target(
        std::uint64_t target_id, const TargetCoordinate& target,
        const AssetPose& pose, const PointingLimits& limits, std::uint64_t now_us);

private:
    StateStore& state_;
    EffectorCommandConfig config_;
    std::mutex mutex_;
    CommandIdentity identity_;
};
}  // namespace c2
