#include "c2/effector_command_service.hpp"

#include "c2/protocol_validation.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

namespace c2 {
namespace {
bool valid_limits(const PointingLimits& limits) noexcept {
    return std::isfinite(limits.minimum_pan_deg) &&
           std::isfinite(limits.maximum_pan_deg) &&
           std::isfinite(limits.minimum_tilt_deg) &&
           std::isfinite(limits.maximum_tilt_deg) &&
           limits.minimum_pan_deg <= limits.maximum_pan_deg &&
           limits.minimum_tilt_deg <= limits.maximum_tilt_deg;
}



EffectorCommandError command_error(const PointingError error) noexcept {
    switch (error) {
        case PointingError::invalid_target:
            return EffectorCommandError::invalid_target;
        case PointingError::invalid_effector_pose:
            return EffectorCommandError::invalid_effector_pose;
        case PointingError::coincident_target:
            return EffectorCommandError::coincident_target;
        case PointingError::outside_turret_limits:
            return EffectorCommandError::outside_turret_limits;
    }
    return EffectorCommandError::invalid_target;
}
}  // namespace

EffectorCommandService::EffectorCommandService(
    StateStore& state, const EffectorCommandConfig config)
    : state_(state),
      config_(config),
      identity_(config.first_command_id, config.first_sequence) {
    if (!valid_limits(config_.limits))
        throw std::invalid_argument("effector pointing limits are invalid");
    if (config_.command_validity_us == 0)
        throw std::invalid_argument("command validity must be non-zero");
}

EffectorCommandResult EffectorCommandService::create_for_target(
    const std::uint64_t target_id, const std::uint64_t now_us) {
    if (now_us == 0) return EffectorCommandError::invalid_time;
    if (target_id > std::numeric_limits<std::uint32_t>::max())
        return EffectorCommandError::target_unavailable;
    if (now_us > std::numeric_limits<std::uint64_t>::max() - config_.command_validity_us)
        return EffectorCommandError::deadline_overflow;

    const auto target = state_.target(static_cast<std::uint32_t>(target_id), now_us);
    if (!target) return EffectorCommandError::target_unavailable;
    const auto effector_pose = state_.asset_pose(ComponentId::effector_asset);
    if (!effector_pose) return EffectorCommandError::effector_pose_unavailable;

    return create_for_target(target_id, *target, *effector_pose, config_.limits, now_us);
}

EffectorCommandResult EffectorCommandService::create_for_target(
    const std::uint64_t target_id, const TargetCoordinate& target,
    const AssetPose& pose, const PointingLimits& limits, const std::uint64_t now_us) {
    if (now_us == 0) return EffectorCommandError::invalid_time;
    if (now_us > std::numeric_limits<std::uint64_t>::max() - config_.command_validity_us)
        return EffectorCommandError::deadline_overflow;
    const auto pointing = calculate_effector_pointing(target, pose, limits);
    if (const auto* error = std::get_if<PointingError>(&pointing))
        return command_error(*error);
    const auto& solution = std::get<PointingSolution>(pointing);

    std::lock_guard lock(mutex_);
    EffectorTurretCommand command{
        {protocol_version, 1, now_us, ComponentId::command_and_control,
         ComponentId::effector_asset},
        1, solution.pan_deg, solution.tilt_deg,
        now_us + config_.command_validity_us};
    if (!validate(command).valid())
        throw std::logic_error("generated effector command is invalid");

    identity_.assign(command);
    return command;
}
}  // namespace c2
