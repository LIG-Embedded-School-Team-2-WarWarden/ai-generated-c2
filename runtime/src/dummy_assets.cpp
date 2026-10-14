#include "c2/dummy_assets.hpp"
#include "c2/pointing.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace c2 {
namespace {
std::uint32_t next(const std::uint32_t value) noexcept {
    return value == std::numeric_limits<std::uint32_t>::max() ? 1U : value + 1U;
}
bool valid_limits(const ObservationTurretLimits& limits) noexcept {
    return std::isfinite(limits.minimum_pan_deg) && std::isfinite(limits.maximum_pan_deg) &&
           std::isfinite(limits.minimum_tilt_deg) && std::isfinite(limits.maximum_tilt_deg) &&
           limits.minimum_pan_deg <= limits.maximum_pan_deg &&
           limits.minimum_tilt_deg <= limits.maximum_tilt_deg;
}
bool in_limits(const float pan, const float tilt, const ObservationTurretLimits& limits) noexcept {
    return pan >= limits.minimum_pan_deg && pan <= limits.maximum_pan_deg &&
           tilt >= limits.minimum_tilt_deg && tilt <= limits.maximum_tilt_deg;
}
bool newer_command(const std::uint32_t candidate, const std::uint32_t current) noexcept {
    return static_cast<std::int32_t>(candidate - current) > 0;
}
}  // namespace

DummyObservationAsset::DummyObservationAsset(
    AssetPose pose, const ObservationTurretLimits limits,
    const std::size_t maximum_cached_results)
    : pose_(std::move(pose)), limits_(limits),
      maximum_cached_results_(maximum_cached_results) {
    if (!validate(pose_).valid() || pose_.header.source_id != ComponentId::observation_asset)
        throw std::invalid_argument("invalid observation pose");
    if (!valid_limits(limits_)) throw std::invalid_argument("invalid observation limits");
    if (maximum_cached_results_ == 0)
        throw std::invalid_argument("maximum cached results must be non-zero");
    asset_id_ = pose_.header.asset_id;
    session_id_ = pose_.header.session_id;
    status_.state = ObservationState::standby;
}

ProcessedCommand DummyObservationAsset::handle(
    const ObservationTurretCommand& command, const std::uint64_t now_us) {
    std::lock_guard lock(mutex_);
    if (command.header.asset_id != asset_id_ ||
        command.header.session_id != session_id_) {
        auto response = ack(command.command_id, CommandResult::rejected,
                            dummy_error::invalid_command, now_us);
        return {response, false,
                error(dummy_error::invalid_command, ErrorSeverity::warning,
                      command.command_id, "command identity mismatch", now_us)};
    }
    if (const auto found = results_.find(command.command_id); found != results_.end())
        return {found->second, true, std::nullopt};
    if (latest_command_id_ && !newer_command(command.command_id, *latest_command_id_)) {
        auto response = ack(command.command_id, CommandResult::rejected,
                            dummy_error::duplicate_command, now_us);
        return {response, true,
                error(dummy_error::duplicate_command, ErrorSeverity::warning,
                      command.command_id, "stale or duplicate observation command", now_us)};
    }
    CommandResult result = CommandResult::completed;
    std::uint32_t error_code{};
    std::string detail;
    if (!validate(command, limits_).valid()) {
        result = CommandResult::rejected;
        error_code = dummy_error::invalid_command;
        detail = "invalid observation command";
    } else if (now_us >= command.valid_until_us) {
        result = CommandResult::rejected;
        error_code = dummy_error::expired_command;
        detail = "expired observation command";
    } else if (command.command_type == ObservationTurretCommandType::scan) {
        status_.current_pan_deg = command.target_pan_deg;
        status_.current_tilt_deg = command.target_tilt_deg;
        status_.state = ObservationState::operating;
        status_.lidar_active = true;
        status_.turret_active = false;
    } else if (command.command_type == ObservationTurretCommandType::absolute_angle) {
        status_.current_pan_deg = command.target_pan_deg;
        status_.current_tilt_deg = command.target_tilt_deg;
        status_.state = ObservationState::standby;
        status_.lidar_active = false;
        status_.turret_active = false;
    } else if (command.command_type == ObservationTurretCommandType::home) {
        status_.current_pan_deg = 0;
        status_.current_tilt_deg = 0;
        status_.state = ObservationState::standby;
        status_.lidar_active = false;
        status_.turret_active = false;
    } else {
        status_.state = ObservationState::standby;
        status_.lidar_active = false;
        status_.turret_active = false;
    }
    const auto response = ack(command.command_id, result, error_code, now_us);
    remember(command.command_id, response);
    std::optional<ErrorReport> report;
    if (error_code != 0)
        report = error(error_code, ErrorSeverity::warning, command.command_id,
                       std::move(detail), now_us);
    return {response, false, std::move(report)};
}

ProcessedCommand DummyObservationAsset::handle(
    const DevelopmentPoseCommand& command, const std::uint64_t now_us) {
    std::lock_guard lock(mutex_);
    if (command.header.asset_id != asset_id_ || command.header.session_id != session_id_ ||
        command.header.destination_id != ComponentId::observation_asset)
        return {ack(command.command_id, CommandResult::rejected,
                    dummy_error::invalid_command, now_us), false, std::nullopt};
    if (const auto found = results_.find(command.command_id); found != results_.end())
        return {found->second, true, std::nullopt};
    if (latest_command_id_ && !newer_command(command.command_id, *latest_command_id_))
        return {ack(command.command_id, CommandResult::rejected,
                    dummy_error::duplicate_command, now_us), true, std::nullopt};
    std::uint32_t code{};
    if (!validate(command).valid()) code = dummy_error::invalid_command;
    else if (now_us >= command.valid_until_us) code = dummy_error::expired_command;
    else if (status_.state != ObservationState::standby || status_.turret_active ||
             status_.lidar_active) code = dummy_error::invalid_state;
    if (code == 0) {
        pose_.x_m = command.x_m;
        pose_.y_m = command.y_m;
        pose_.z_m = command.z_m;
        pose_.azimuth_deg = command.azimuth_deg;
    }
    const auto response = ack(command.command_id,
        code == 0 ? CommandResult::completed : CommandResult::rejected, code, now_us);
    remember(command.command_id, response);
    return {response, false, code == 0 ? std::nullopt : std::optional<ErrorReport>{
        error(code, ErrorSeverity::warning, command.command_id,
              "development pose rejected", now_us)}};
}

MessageHeader DummyObservationAsset::header(const std::uint64_t now_us) {
    auto value = MessageHeader{protocol_version, sequence_, now_us,
                               ComponentId::observation_asset,
                               ComponentId::command_and_control,
                               asset_id_, session_id_};
    sequence_ = next(sequence_);
    return value;
}
CommandAck DummyObservationAsset::ack(
    const std::uint32_t id, const CommandResult result,
    const std::uint32_t error_code, const std::uint64_t now_us) {
    return {header(now_us), id, result, error_code, now_us};
}
ErrorReport DummyObservationAsset::error(
    const std::uint32_t error_code, const ErrorSeverity severity,
    const std::uint32_t command_id, std::string detail, const std::uint64_t now_us) {
    return {header(now_us), error_code, severity, command_id, now_us, std::move(detail)};
}
void DummyObservationAsset::remember(
    const std::uint32_t command_id, const CommandAck& result) {
    results_.emplace(command_id, result);
    result_order_.push_back(command_id);
    latest_command_id_ = command_id;
    while (result_order_.size() > maximum_cached_results_) {
        results_.erase(result_order_.front());
        result_order_.pop_front();
    }
}
AssetPose DummyObservationAsset::asset_pose(const std::uint64_t now_us) {
    std::lock_guard lock(mutex_);
    auto value = pose_;
    value.header = header(now_us);
    return value;
}
ObservationStatus DummyObservationAsset::status(const std::uint64_t now_us) {
    std::lock_guard lock(mutex_);
    auto value = status_;
    value.header = header(now_us);
    value.timestamp_us = now_us;
    return value;
}
Heartbeat DummyObservationAsset::heartbeat(
    const std::uint64_t now_us, const std::uint64_t uptime_ms) {
    std::lock_guard lock(mutex_);
    const auto state = status_.state == ObservationState::fault
                           ? AssetOperatingState::fault : AssetOperatingState::operating;
    return {header(now_us), state, uptime_ms, now_us};
}
TargetCoordinate DummyObservationAsset::target(
    const std::uint32_t id, const float x, const float y, const float z,
    const float confidence, const std::uint64_t now_us) {
    std::lock_guard lock(mutex_);
    return {header(now_us), id, now_us, CoordinateFrame::project_frame,
            x, y, z, confidence, 0, 0, 0, false};
}
TargetCoordinate DummyObservationAsset::target(
    const std::uint32_t id, const float x, const float y, const float z,
    const float vx, const float vy, const float vz, const bool velocity_valid,
    const float confidence, const std::uint64_t now_us) {
    std::lock_guard lock(mutex_);
    return {header(now_us), id, now_us, CoordinateFrame::project_frame,
            x, y, z, confidence, vx, vy, vz, velocity_valid};
}
bool DummyObservationAsset::observe_control_heartbeat(
    const Heartbeat& value, const std::uint64_t received_at_us) {
    std::lock_guard lock(mutex_);
    if (!validate(value).valid() || value.header.source_id != ComponentId::command_and_control ||
        value.header.destination_id != ComponentId::observation_asset ||
        value.header.asset_id != asset_id_ ||
        value.header.session_id != session_id_ || received_at_us == 0)
        return false;
    last_control_heartbeat_us_ = received_at_us;
    watchdog_tripped_ = false;
    return true;
}
std::optional<ErrorReport> DummyObservationAsset::check_watchdog(
    const std::uint64_t now_us, const std::uint64_t timeout_us) {
    std::lock_guard lock(mutex_);
    if (timeout_us == 0 || last_control_heartbeat_us_ == 0 ||
        now_us <= last_control_heartbeat_us_ ||
        now_us - last_control_heartbeat_us_ <= timeout_us || watchdog_tripped_)
        return std::nullopt;
    status_.state = ObservationState::standby;
    status_.lidar_active = false;
    status_.turret_active = false;
    watchdog_tripped_ = true;
    return error(dummy_error::communication_timeout, ErrorSeverity::warning, 0,
                 "command-and-control heartbeat timed out", now_us);
}

DummyEffectorAsset::DummyEffectorAsset(
    AssetPose pose, const ObservationTurretLimits limits,
    const std::size_t maximum_cached_results)
    : pose_(std::move(pose)), limits_(limits),
      maximum_cached_results_(maximum_cached_results) {
    if (!validate(pose_).valid() || pose_.header.source_id != ComponentId::effector_asset)
        throw std::invalid_argument("invalid effector pose");
    if (!valid_limits(limits_)) throw std::invalid_argument("invalid effector limits");
    if (maximum_cached_results_ == 0)
        throw std::invalid_argument("maximum cached results must be non-zero");
    asset_id_ = pose_.header.asset_id;
    session_id_ = pose_.header.session_id;
    status_.state = EffectorState::standby;
}

ProcessedCommand DummyEffectorAsset::handle(
    const EffectorTurretCommand& command, const std::uint64_t now_us) {
    std::lock_guard lock(mutex_);
    if (command.header.asset_id != asset_id_ ||
        command.header.session_id != session_id_) {
        auto response = ack(command.command_id, CommandResult::rejected,
                            dummy_error::invalid_command, now_us);
        return {response, false,
                error(dummy_error::invalid_command, ErrorSeverity::warning,
                      command.command_id, "command identity mismatch", now_us)};
    }
    if (const auto found = results_.find(command.command_id); found != results_.end())
        return {found->second, true, std::nullopt};
    if (latest_command_id_ && !newer_command(command.command_id, *latest_command_id_)) {
        auto response = ack(command.command_id, CommandResult::rejected,
                            dummy_error::duplicate_command, now_us);
        return {response, true,
                error(dummy_error::duplicate_command, ErrorSeverity::warning,
                      command.command_id, "stale or duplicate effector command", now_us)};
    }
    CommandResult result = CommandResult::completed;
    std::uint32_t error_code{};
    std::string detail;
    if (!validate(command).valid()) {
        result = CommandResult::rejected;
        error_code = dummy_error::invalid_command;
        detail = "invalid effector pointing command";
    } else if (!in_limits(command.target_pan_deg, command.target_tilt_deg, limits_)) {
        result = CommandResult::rejected;
        error_code = dummy_error::out_of_range;
        detail = "effector pointing command exceeds turret limits";
    } else if (now_us >= command.valid_until_us) {
        result = CommandResult::rejected;
        error_code = dummy_error::expired_command;
        detail = "expired effector pointing command";
    } else {
        // Manual pointing does not designate an output target or retain tracking.
        safe_stop(TrackingStopReason::operator_stop, 0);
        status_.tracking_track_id = 0;
        status_.target_pan_deg = command.target_pan_deg;
        status_.target_tilt_deg = command.target_tilt_deg;
        status_.current_pan_deg = command.target_pan_deg;
        status_.current_tilt_deg = command.target_tilt_deg;
        status_.aligned = true;
        status_.state = EffectorState::ready;
    }
    const auto response = ack(command.command_id, result, error_code, now_us);
    remember(command.command_id, response);
    std::optional<ErrorReport> report;
    if (error_code != 0)
        report = error(error_code, ErrorSeverity::warning, command.command_id,
                       std::move(detail), now_us);
    return {response, false, std::move(report)};
}

ProcessedCommand DummyEffectorAsset::handle(
    const AttackCommand& command, const std::uint64_t now_us) {
    std::lock_guard lock(mutex_);
    if (command.header.asset_id != asset_id_ ||
        command.header.session_id != session_id_) {
        auto response = ack(command.command_id, CommandResult::rejected,
                            dummy_error::invalid_command, now_us);
        return {response, false,
                error(dummy_error::invalid_command, ErrorSeverity::error,
                      command.command_id, "command identity mismatch", now_us)};
    }
    if (const auto found = results_.find(command.command_id); found != results_.end())
        return {found->second, true, std::nullopt};
    if (latest_command_id_ && !newer_command(command.command_id, *latest_command_id_)) {
        auto response = ack(command.command_id, CommandResult::rejected,
                            dummy_error::duplicate_command, now_us);
        return {response, true,
                error(dummy_error::duplicate_command, ErrorSeverity::warning,
                      command.command_id, "stale or duplicate attack command", now_us)};
    }
    advance(now_us);
    CommandResult result = CommandResult::completed;
    std::uint32_t error_code{};
    std::string detail;
    if (!validate(command).valid()) {
        result = CommandResult::rejected;
        error_code = dummy_error::invalid_command;
        detail = "invalid attack command";
    } else if (now_us >= command.valid_until_us) {
        result = CommandResult::rejected;
        error_code = dummy_error::expired_command;
        detail = "expired attack command";
    } else if (command.action == AttackAction::emergency_stop) {
        safe_stop(TrackingStopReason::emergency_stop, 0);
    } else if (command.action == AttackAction::stop) {
        safe_stop(TrackingStopReason::operator_stop, 0);
    } else if (status_.state != EffectorState::ready) {
        result = CommandResult::rejected;
        error_code = dummy_error::invalid_state;
        detail = "effector is not ready";
    } else if (!status_.aligned || !target_update_ || now_us >= target_update_->valid_until_us ||
               command.target_id != target_update_->track_id) {
        result = CommandResult::rejected;
        error_code = dummy_error::not_aligned;
        detail = "effector is not aligned with the requested target";
    } else if (command.action == AttackAction::arm) {
        status_.attack_armed = true;
    } else if (!status_.attack_armed) {
        result = CommandResult::rejected;
        error_code = dummy_error::not_armed;
        detail = "effector is not armed";
    } else {
        status_.attack_active = true;
        status_.automatic_tracking_active = true;
        status_.tracking_track_id = target_update_->track_id;
        status_.tracking_stop_reason = TrackingStopReason::none;
        status_.state = EffectorState::active;
        const auto duration_us = static_cast<std::uint64_t>(command.duration_ms) * 1000U;
        attack_end_us_ = now_us > std::numeric_limits<std::uint64_t>::max() - duration_us
                             ? std::numeric_limits<std::uint64_t>::max()
                             : now_us + duration_us;
    }
    const auto response = ack(command.command_id, result, error_code, now_us);
    remember(command.command_id, response);
    std::optional<ErrorReport> report;
    if (error_code != 0)
        report = error(error_code, ErrorSeverity::error, command.command_id,
                       std::move(detail), now_us);
    return {response, false, std::move(report)};
}

bool DummyEffectorAsset::handle(
    const TargetTrackUpdate& update, const std::uint64_t now_us) {
    std::lock_guard lock(mutex_);
    if (!validate(update).valid() || now_us == 0 || now_us >= update.valid_until_us ||
        update.header.asset_id != asset_id_ || update.header.session_id != session_id_)
        return false;
    if (target_update_) {
        if (update.track_id != target_update_->track_id) return false;
        if (update.measurement_time_us < target_update_->measurement_time_us ||
            (update.measurement_time_us == target_update_->measurement_time_us &&
             update.header.sequence <= target_update_->header.sequence)) {
            return false;
        }
    }
    target_update_ = update;
    status_.tracking_track_id = update.track_id;
    status_.last_target_measurement_time_us = update.measurement_time_us;
    status_.tracking_stop_reason = TrackingStopReason::none;
    status_.error_code = 0;
    return true;
}

std::optional<ErrorReport> DummyEffectorAsset::control_step(
    const std::uint64_t now_us, const std::uint64_t maximum_prediction_us) {
    std::lock_guard lock(mutex_);
    advance(now_us);
    if (!target_update_) return std::nullopt;
    const auto& update = *target_update_;
    if (pose_.header.timestamp_us == 0 || now_us < pose_.header.timestamp_us ||
        now_us - pose_.header.timestamp_us > 3'000'000) {
        safe_stop(TrackingStopReason::pose_unavailable,
                  dummy_error::invalid_target_state);
        return error(dummy_error::invalid_target_state, ErrorSeverity::critical, 0,
                     "effector pose is missing or stale", now_us);
    }
    if (now_us < update.measurement_time_us) {
        safe_stop(TrackingStopReason::invalid_target, dummy_error::invalid_target_state);
        return error(dummy_error::invalid_target_state, ErrorSeverity::critical, 0,
                     "target measurement timestamp is in the future", now_us);
    }
    const auto age = now_us - update.measurement_time_us;
    status_.target_freshness_us = age;
    if (now_us >= update.valid_until_us) {
        safe_stop(TrackingStopReason::target_expired, dummy_error::target_expired);
        return error(dummy_error::target_expired, ErrorSeverity::critical, 0,
                     "target track update expired", now_us);
    }
    if (maximum_prediction_us == 0 || age > maximum_prediction_us) {
        safe_stop(TrackingStopReason::prediction_timeout, dummy_error::target_expired);
        return error(dummy_error::target_expired, ErrorSeverity::critical, 0,
                     "maximum dead-reckoning interval exceeded", now_us);
    }
    const auto seconds = static_cast<double>(age) / 1'000'000.0;
    TargetCoordinate predicted{{protocol_version, 1, now_us,
        ComponentId::observation_asset, ComponentId::command_and_control,
        update.observation_asset_id, update.observation_session_id},
        1, now_us, CoordinateFrame::project_frame,
        static_cast<float>(update.x_m + update.vx_mps * seconds),
        static_cast<float>(update.y_m + update.vy_mps * seconds),
        static_cast<float>(update.z_m + update.vz_mps * seconds),
        update.confidence, update.vx_mps, update.vy_mps, update.vz_mps, true};
    status_.predicted_x_m = predicted.x_m;
    status_.predicted_y_m = predicted.y_m;
    status_.predicted_z_m = predicted.z_m;
    const auto solution = calculate_effector_pointing(
        predicted, pose_, {limits_.minimum_pan_deg, limits_.maximum_pan_deg,
                           limits_.minimum_tilt_deg, limits_.maximum_tilt_deg});
    if (!std::holds_alternative<PointingSolution>(solution)) {
        safe_stop(TrackingStopReason::outside_turret_limits, dummy_error::out_of_range);
        return error(dummy_error::out_of_range, ErrorSeverity::critical, 0,
                     "predicted target cannot be pointed within turret limits", now_us);
    }
    const auto& pointing = std::get<PointingSolution>(solution);
    status_.target_pan_deg = pointing.pan_deg;
    status_.target_tilt_deg = pointing.tilt_deg;
    status_.current_pan_deg = pointing.pan_deg;
    status_.current_tilt_deg = pointing.tilt_deg;
    status_.aligned = true;
    if (!status_.attack_active) status_.state = EffectorState::ready;
    return std::nullopt;
}

void DummyEffectorAsset::advance(const std::uint64_t now_us) {
    if (status_.attack_active && now_us >= attack_end_us_) {
        status_.attack_active = false;
        status_.attack_armed = false;
        status_.state = EffectorState::ready;
    }
}
void DummyEffectorAsset::safe_stop(
    const TrackingStopReason reason, const std::uint32_t error_code) {
    status_.attack_active = false;
    status_.attack_armed = false;
    status_.automatic_tracking_active = false;
    status_.aligned = false;
    status_.state = error_code == 0 ? EffectorState::standby : EffectorState::fault;
    status_.tracking_stop_reason = reason;
    status_.error_code = error_code;
    attack_end_us_ = 0;
    if (reason == TrackingStopReason::operator_stop ||
        reason == TrackingStopReason::emergency_stop)
        target_update_.reset();
}
ProcessedCommand DummyEffectorAsset::handle(
    const DevelopmentPoseCommand& command, const std::uint64_t now_us) {
    std::lock_guard lock(mutex_);
    if (command.header.asset_id != asset_id_ || command.header.session_id != session_id_ ||
        command.header.destination_id != ComponentId::effector_asset)
        return {ack(command.command_id, CommandResult::rejected,
                    dummy_error::invalid_command, now_us), false, std::nullopt};
    if (const auto found = results_.find(command.command_id); found != results_.end())
        return {found->second, true, std::nullopt};
    if (latest_command_id_ && !newer_command(command.command_id, *latest_command_id_))
        return {ack(command.command_id, CommandResult::rejected,
                    dummy_error::duplicate_command, now_us), true, std::nullopt};
    std::uint32_t code{};
    if (!validate(command).valid()) code = dummy_error::invalid_command;
    else if (now_us >= command.valid_until_us) code = dummy_error::expired_command;
    else if (status_.state != EffectorState::standby || status_.attack_armed ||
             status_.attack_active || status_.automatic_tracking_active)
        code = dummy_error::invalid_state;
    if (code == 0) {
        pose_.x_m = command.x_m;
        pose_.y_m = command.y_m;
        pose_.z_m = command.z_m;
        pose_.azimuth_deg = command.azimuth_deg;
        target_update_.reset();
        status_.aligned = false;
        status_.tracking_track_id = 0;
    }
    const auto response = ack(command.command_id,
        code == 0 ? CommandResult::completed : CommandResult::rejected, code, now_us);
    remember(command.command_id, response);
    return {response, false, code == 0 ? std::nullopt : std::optional<ErrorReport>{
        error(code, ErrorSeverity::warning, command.command_id,
              "development pose rejected", now_us)}};
}

MessageHeader DummyEffectorAsset::header(const std::uint64_t now_us) {
    auto value = MessageHeader{protocol_version, sequence_, now_us,
                               ComponentId::effector_asset,
                               ComponentId::command_and_control,
                               asset_id_, session_id_};
    sequence_ = next(sequence_);
    return value;
}
CommandAck DummyEffectorAsset::ack(
    const std::uint32_t id, const CommandResult result,
    const std::uint32_t error_code, const std::uint64_t now_us) {
    return {header(now_us), id, result, error_code, now_us};
}
ErrorReport DummyEffectorAsset::error(
    const std::uint32_t error_code, const ErrorSeverity severity,
    const std::uint32_t command_id, std::string detail, const std::uint64_t now_us) {
    return {header(now_us), error_code, severity, command_id, now_us, std::move(detail)};
}
void DummyEffectorAsset::remember(
    const std::uint32_t command_id, const CommandAck& result) {
    results_.emplace(command_id, result);
    result_order_.push_back(command_id);
    latest_command_id_ = command_id;
    while (result_order_.size() > maximum_cached_results_) {
        results_.erase(result_order_.front());
        result_order_.pop_front();
    }
}
AssetPose DummyEffectorAsset::asset_pose(const std::uint64_t now_us) {
    std::lock_guard lock(mutex_);
    pose_.header = header(now_us);
    return pose_;
}
EffectorStatus DummyEffectorAsset::status(const std::uint64_t now_us) {
    std::lock_guard lock(mutex_);
    advance(now_us);
    auto value = status_;
    value.header = header(now_us);
    value.timestamp_us = now_us;
    return value;
}
Heartbeat DummyEffectorAsset::heartbeat(
    const std::uint64_t now_us, const std::uint64_t uptime_ms) {
    std::lock_guard lock(mutex_);
    advance(now_us);
    auto state = AssetOperatingState::standby;
    if (status_.state == EffectorState::ready) state = AssetOperatingState::ready;
    else if (status_.state == EffectorState::active) state = AssetOperatingState::active;
    else if (status_.state == EffectorState::fault) state = AssetOperatingState::fault;
    return {header(now_us), state, uptime_ms, now_us};
}
bool DummyEffectorAsset::observe_control_heartbeat(
    const Heartbeat& value, const std::uint64_t received_at_us) {
    std::lock_guard lock(mutex_);
    if (!validate(value).valid() || value.header.source_id != ComponentId::command_and_control ||
        value.header.destination_id != ComponentId::effector_asset ||
        value.header.asset_id != asset_id_ ||
        value.header.session_id != session_id_ || received_at_us == 0)
        return false;
    last_control_heartbeat_us_ = received_at_us;
    watchdog_tripped_ = false;
    return true;
}
std::optional<ErrorReport> DummyEffectorAsset::check_watchdog(
    const std::uint64_t now_us, const std::uint64_t timeout_us) {
    std::lock_guard lock(mutex_);
    if (timeout_us == 0 || last_control_heartbeat_us_ == 0 ||
        now_us <= last_control_heartbeat_us_ ||
        now_us - last_control_heartbeat_us_ <= timeout_us || watchdog_tripped_)
        return std::nullopt;
    safe_stop(TrackingStopReason::communication_timeout,
              dummy_error::communication_timeout);
    watchdog_tripped_ = true;
    return error(dummy_error::communication_timeout, ErrorSeverity::critical, 0,
                 "command-and-control heartbeat timed out", now_us);
}
}  // namespace c2
