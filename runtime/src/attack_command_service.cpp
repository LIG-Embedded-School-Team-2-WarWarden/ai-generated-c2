#include "c2/attack_command_service.hpp"

#include "c2/protocol_validation.hpp"

#include <limits>
#include <stdexcept>

namespace c2 {
namespace {
bool supported(const AttackAction action) noexcept {
    return action == AttackAction::arm || action == AttackAction::start ||
           action == AttackAction::stop || action == AttackAction::emergency_stop;
}

bool safety_override(const AttackAction action) noexcept {
    return action == AttackAction::stop || action == AttackAction::emergency_stop;
}
}  // namespace

AttackCommandService::AttackCommandService(const AttackCommandConfig config)
    : config_(config),
      identity_(config.first_command_id, config.first_sequence) {
    if (config_.command_validity_us == 0)
        throw std::invalid_argument("command validity must be non-zero");
}

AttackStatusUpdateResult AttackCommandService::update_status(
    const EffectorStatus& status) {
    if (!validate(status).valid()) return AttackStatusUpdateResult::invalid;
    std::lock_guard lock(mutex_);
    if (status_) {
        if (status.header.sequence == status_->header.sequence &&
            status.timestamp_us == status_->timestamp_us)
            return AttackStatusUpdateResult::duplicate;
        if (status.timestamp_us <= status_->timestamp_us)
            return AttackStatusUpdateResult::out_of_order;
    }
    status_ = status;
    return AttackStatusUpdateResult::stored;
}

AttackCommandResult AttackCommandService::create(
    const AttackAction action, std::uint64_t target_id,
    std::uint32_t duration_ms, const std::uint64_t now_us) {
    std::lock_guard lock(mutex_);
    return create_locked(action, target_id, duration_ms, now_us,
        {status_, status_ && status_->tracking_track_id != 0 &&
             now_us >= status_->timestamp_us &&
             now_us - status_->timestamp_us < config_.command_validity_us,
         status_ ? status_->tracking_track_id : 0});
}

AttackCommandResult AttackCommandService::create(
    const AttackAction action, const std::uint64_t target_id,
    const std::uint32_t duration_ms, const std::uint64_t now_us,
    const AttackCommandContext& context) {
    std::lock_guard lock(mutex_);
    return create_locked(action, target_id, duration_ms, now_us, context);
}

AttackCommandResult AttackCommandService::create_locked(
    const AttackAction action, std::uint64_t target_id,
    std::uint32_t duration_ms, const std::uint64_t now_us,
    const AttackCommandContext& context) {
    if (!supported(action)) return AttackCommandError::unsupported_action;
    if (now_us == 0) return AttackCommandError::invalid_time;
    if (now_us > std::numeric_limits<std::uint64_t>::max() - config_.command_validity_us)
        return AttackCommandError::deadline_overflow;

    if (!safety_override(action)) {
        if (!context.status) return AttackCommandError::status_unavailable;
        if (!context.target_available) return AttackCommandError::target_unavailable;
        if (target_id == 0 || target_id != context.target_id)
            return AttackCommandError::target_mismatch;
        if (context.status->state != EffectorState::ready)
            return AttackCommandError::invalid_state;
        if (!context.status->aligned) return AttackCommandError::not_aligned;
        if (action == AttackAction::start && !context.status->attack_armed)
            return AttackCommandError::not_armed;
        if (action == AttackAction::start && duration_ms == 0)
            return AttackCommandError::invalid_duration;
    } else {
        target_id = 0;
        duration_ms = 0;
    }

    AttackCommand command{
        {protocol_version, 1, now_us, ComponentId::command_and_control,
         ComponentId::effector_asset},
        1, target_id, action, duration_ms,
        now_us + config_.command_validity_us};
    if (!validate(command).valid())
        throw std::logic_error("generated attack command is invalid");
    identity_.assign(command);
    return command;
}
}  // namespace c2
