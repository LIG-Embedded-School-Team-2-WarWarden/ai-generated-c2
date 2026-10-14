#include "c2/server_runtime.hpp"

#include "c2/protobuf_codec.hpp"
#include "c2/protocol_validation.hpp"

#include <stdexcept>
#include <utility>
#include <limits>

namespace c2 {
ServerRuntime::ServerRuntime(ServerRuntimeConfig config, DatagramSender sender)
    : config_(std::move(config)),
      sender_(std::move(sender)),
      registry_(config_.registry),
      tracks_(config_.tracks),
      command_tracker_(config_.commands, config_.event_sink),
      assignments_(config_.assignments),
      state_(config_.state),
      telemetry_(config_.maximum_error_history),
      connections_(config_.connections),
      observation_commands_(config_.observation_commands),
      effector_commands_(state_, config_.effector_commands),
      attack_commands_(config_.attack_commands),
      effector_identity_(config_.effector_commands.first_command_id,
                         config_.effector_commands.first_sequence),
      track_updates_(sender_, config_.event_sink),
      heartbeats_(sender_),
      assignment_coordinator_(registry_, tracks_, assignments_, track_updates_, config_.assignments),
      legacy_commands_(config_.command_ack_timeout_us, config_.command_max_attempts),
      rejection_log_(config_.maximum_inbound_rejections),
      legacy_inbound_(state_, telemetry_, connections_, attack_commands_, legacy_commands_),
      asset_inbound_(registry_, tracks_, command_tracker_, assignments_, legacy_inbound_,
          track_updates_, rejection_log_, asset_lifecycle_mutex_,
          config_.emergency_stop_repetitions,
          [this](const auto& asset, auto action, auto repetitions, auto now_us) {
              return dispatch_safety_command(asset, action, repetitions, now_us);
          }) {
    if (!sender_) throw std::invalid_argument("datagram sender must be set");
    if (config_.emergency_stop_repetitions == 0)
        throw std::invalid_argument("emergency stop repetitions must be non-zero");
}

InboundResult ServerRuntime::ingest(
    const std::span<const std::byte> datagram, const Endpoint& source,
    const std::uint64_t received_at_us) {
    const auto decoded = protobuf::decode(datagram);
    if (!std::holds_alternative<Envelope>(decoded)) {
        rejection_log_.record(InboundRejectionCategory::invalid_packet,
            MessageKind::unspecified, nullptr, AssetRegistryResult::invalid, received_at_us);
        emit_event(config_.event_sink, {"invalid_packet", received_at_us, 0, 0, 0, "decode"});
        return InboundResult::invalid_packet;
    }
    const auto& envelope = std::get<Envelope>(decoded);
    const auto result = asset_inbound_.receive(envelope, source, received_at_us);
    audit_inbound(envelope, result, received_at_us);
    return result;
}

DevelopmentPoseDispatchResult ServerRuntime::set_development_pose(
    const std::uint64_t asset_id, const float x_m, const float y_m,
    const float z_m, const float azimuth_deg, const std::uint64_t now_us) {
    std::lock_guard lifecycle_lock(asset_lifecycle_mutex_);
    const auto asset = registry_.asset(asset_id, now_us);
    if (!asset || asset->connection_state != AssetConnectionState::connected)
        return DispatchError::connection_unavailable;
    if ((asset->capabilities & capability::development_pose) == 0)
        return DispatchError::command_rejected;
    if (asset->role == AssetRole::effector) {
        for (const auto& assignment : assignments_.assignments())
            if (assignment.effector_asset_id == asset_id &&
                assignment.state == AssignmentResult::assigned)
                return DispatchError::command_rejected;
    }
    const auto validity = asset->role == AssetRole::observation
        ? config_.observation_commands.command_validity_us
        : config_.effector_commands.command_validity_us;
    if (now_us == 0 || now_us > std::numeric_limits<std::uint64_t>::max() - validity)
        return DispatchError::command_rejected;
    DevelopmentPoseCommand command{
        {protocol_version, 1, now_us, ComponentId::command_and_control,
         asset->role == AssetRole::observation ? ComponentId::observation_asset
                                               : ComponentId::effector_asset,
         asset_id, asset->session_id},
        1, CoordinateFrame::project_frame, x_m, y_m, z_m, azimuth_deg, now_us + validity};
    if (!validate(command).valid()) return DispatchError::command_rejected;
    try {
        if (asset->role == AssetRole::observation) {
            observation_commands_.assign_identity(command);
        } else {
            effector_identity_.assign(command);
        }
        const auto encoded = protobuf::encode(Envelope{command});
        if (command_tracker_.track({asset_id, asset->session_id, command.command_id,
                                   encoded, asset->command_endpoint, now_us,
                                   command.valid_until_us}) != CommandTrackResult::tracked)
            return DispatchError::command_rejected;
        if (!send_tracked(encoded, asset->command_endpoint,
                          {asset->asset_id, asset->session_id, command.command_id}, now_us))
            return DispatchError::delivery_uncertain;
        return command;
    } catch (const std::exception&) {
        return DispatchError::command_rejected;
    }
}

ObservationDispatchResult ServerRuntime::command_observation(
    const std::uint64_t asset_id, const ObservationTurretCommandType type,
    const float pan_deg, const float tilt_deg, const std::uint64_t now_us) {
    std::lock_guard lifecycle_lock(asset_lifecycle_mutex_);
    const auto asset = registry_.asset(asset_id, now_us);
    if (!asset || asset->role != AssetRole::observation ||
        asset->connection_state != AssetConnectionState::connected)
        return DispatchError::connection_unavailable;
    if (!asset->pose_synchronized)
        return DispatchError::pose_resynchronization_required;
    try {
        auto command = observation_commands_.create(
            type, pan_deg, tilt_deg, now_us);
        command.header.asset_id = asset->asset_id;
        command.header.session_id = asset->session_id;
        auto encoded = protobuf::encode(Envelope{command});
        const auto tracked = command_tracker_.track(
            c2::PendingCommand{asset->asset_id, asset->session_id,
                               command.command_id, encoded,
                               asset->command_endpoint, now_us,
                               command.valid_until_us});
        if (tracked != CommandTrackResult::tracked)
            return DispatchError::command_rejected;
        if (!send_tracked(encoded, asset->command_endpoint,
                          {asset->asset_id, asset->session_id, command.command_id}, now_us))
            return DispatchError::delivery_uncertain;
        return command;
    } catch (const std::exception&) {
        return DispatchError::command_rejected;
    }
}

InboundResult ServerRuntime::ingest(
    const std::span<const std::byte> datagram, const std::uint64_t received_at_us) {
    const auto decoded = protobuf::decode(datagram);
    if (!std::holds_alternative<Envelope>(decoded)) {
        emit_event(config_.event_sink, {"invalid_packet", received_at_us, 0, 0, 0, "legacy decode"});
        return InboundResult::invalid_packet;
    }
    const auto& envelope = std::get<Envelope>(decoded);
    const auto result = legacy_inbound_.receive(envelope, received_at_us);
    audit_inbound(envelope, result, received_at_us);
    return result;
}

ObservationDispatchResult ServerRuntime::command_observation(
    const ObservationTurretCommandType type, const float pan_deg,
    const float tilt_deg, const std::uint64_t now_us) {
    if (const auto error = connection_error(ComponentId::observation_asset, now_us))
        return *error;
    try {
        auto command = observation_commands_.create(type, pan_deg, tilt_deg, now_us);
        if (!dispatch_tracked(command, config_.observation_endpoint,
                              ComponentId::observation_asset, now_us))
            return DispatchError::delivery_uncertain;
        return command;
    } catch (const std::exception&) {
        return DispatchError::command_rejected;
    }
}

EffectorDispatchResult ServerRuntime::point_effector(
    const std::uint64_t target_id, const std::uint64_t now_us) {
    std::lock_guard lifecycle_lock(asset_lifecycle_mutex_);
    auto assignment = assignments_.assignment(target_id);
    if (!assignment && tracks_.track(target_id, now_us)) {
        const auto decision = assignment_coordinator_.assign(target_id, now_us);
        if (decision.result != AssignmentResult::assigned)
            return DispatchError::command_rejected;
        assignment = decision.assignment;
    }
    if (assignment && assignment->state == AssignmentResult::assigned) {
        const auto track = tracks_.track(target_id, now_us);
        const auto asset = registry_.asset(assignment->effector_asset_id, now_us);
        if (!track || !asset || asset->session_id != assignment->effector_session_id ||
            asset->connection_state != AssetConnectionState::connected)
            return DispatchError::connection_unavailable;
        if (!asset->pose_synchronized || !asset->pose)
            return DispatchError::pose_resynchronization_required;
        const auto result = effector_commands_.create_for_target(
            target_id, track->measurement, *asset->pose,
            {asset->turret_limits.minimum_pan_deg,
             asset->turret_limits.maximum_pan_deg,
             asset->turret_limits.minimum_tilt_deg,
             asset->turret_limits.maximum_tilt_deg}, now_us);
        if (!std::holds_alternative<EffectorTurretCommand>(result))
            return DispatchError::command_rejected;
        auto command = std::get<EffectorTurretCommand>(result);
        command.header.asset_id = asset->asset_id;
        command.header.session_id = asset->session_id;
        effector_identity_.assign(command);
        const auto encoded = protobuf::encode(Envelope{command});
        if (command_tracker_.track({asset->asset_id, asset->session_id,
                                    command.command_id, encoded,
                                    asset->command_endpoint, now_us,
                                    command.valid_until_us}) !=
            CommandTrackResult::tracked)
            return DispatchError::command_rejected;
        if (!send_tracked(encoded, asset->command_endpoint,
                          {asset->asset_id, asset->session_id, command.command_id}, now_us))
            return DispatchError::delivery_uncertain;
        return command;
    }
    if (const auto error = connection_error(ComponentId::effector_asset, now_us))
        return *error;
    const auto result = effector_commands_.create_for_target(target_id, now_us);
    if (!std::holds_alternative<EffectorTurretCommand>(result))
        return DispatchError::command_rejected;
    auto command = std::get<EffectorTurretCommand>(result);
    effector_identity_.assign(command);
    if (!dispatch_tracked(command, config_.effector_endpoint,
                          ComponentId::effector_asset, now_us))
        return DispatchError::delivery_uncertain;
    return command;
}

AttackDispatchResult ServerRuntime::attack(
    const AttackAction action, const std::uint64_t target_id,
    const std::uint32_t duration_ms, const std::uint64_t now_us) {
    std::lock_guard lifecycle_lock(asset_lifecycle_mutex_);
    if ((action == AttackAction::arm || action == AttackAction::start) &&
        assignments_.assignment(target_id)) {
        const auto assignment = assignments_.assignment(target_id);
        const auto track = tracks_.track(target_id, now_us);
        const auto asset = assignment
            ? registry_.asset(assignment->effector_asset_id, now_us)
            : std::nullopt;
        if (!assignment || assignment->state != AssignmentResult::assigned ||
            !track || !asset ||
            asset->session_id != assignment->effector_session_id ||
            asset->connection_state != AssetConnectionState::connected)
            return DispatchError::connection_unavailable;
        if (!asset->pose_synchronized || !asset->pose)
            return DispatchError::pose_resynchronization_required;
        if (!asset->effector_status || !asset->status_current ||
            asset->effector_status->error_code != 0)
            return DispatchError::command_rejected;
        if (asset->effector_status->tracking_track_id != target_id)
            return DispatchError::command_rejected;
        const auto result = attack_commands_.create(
            action, target_id, duration_ms, now_us,
            {*asset->effector_status, true, track->track_id});
        if (!std::holds_alternative<AttackCommand>(result))
            return DispatchError::command_rejected;
        auto command = std::get<AttackCommand>(result);
        command.header.asset_id = asset->asset_id;
        command.header.session_id = asset->session_id;
        effector_identity_.assign(command);
        if (!track_updates_.publish(*track, *asset, now_us))
            return DispatchError::command_rejected;
        const auto encoded = protobuf::encode(Envelope{command});
        if (command_tracker_.track({asset->asset_id, asset->session_id,
                                    command.command_id, encoded,
                                    asset->command_endpoint, now_us,
                                    command.valid_until_us}) !=
            CommandTrackResult::tracked)
            return DispatchError::command_rejected;
        if (action == AttackAction::start && !assignments_.mark_attack_started(target_id)) {
            (void)command_tracker_.cancel_before_send(
                {asset->asset_id, asset->session_id, command.command_id}, now_us);
            return DispatchError::command_rejected;
        }
        // This is a conservative "may have started" latch requiring explicit safety handling:
        // a throwing sender may have handed the datagram to the OS already.
        if (!send_tracked(encoded, asset->command_endpoint,
                          {asset->asset_id, asset->session_id, command.command_id}, now_us))
            return DispatchError::delivery_uncertain;
        return command;
    }
    if (action != AttackAction::stop && action != AttackAction::emergency_stop) {
        if (const auto error = connection_error(ComponentId::effector_asset, now_us))
            return *error;
    }
    const auto result = attack_commands_.create(action, target_id, duration_ms, now_us);
    if (!std::holds_alternative<AttackCommand>(result))
        return DispatchError::command_rejected;
    auto command = std::get<AttackCommand>(result);
    effector_identity_.assign(command);
    const auto repetitions = action == AttackAction::emergency_stop
                                 ? config_.emergency_stop_repetitions
                                 : 1U;
    if (action == AttackAction::emergency_stop) {
        bool uncertain{};
        for (std::uint32_t attempt = 0; attempt < repetitions; ++attempt)
            if (!send_safety(protobuf::encode(Envelope{command}), config_.effector_endpoint,
                             {0, 0, command.command_id}, now_us))
                uncertain = true;
        if (uncertain) return DispatchError::delivery_uncertain;
    } else {
        if (!dispatch_tracked(command, config_.effector_endpoint,
                              ComponentId::effector_asset, now_us))
            return DispatchError::delivery_uncertain;
    }
    return command;
}

AttackDispatchResult ServerRuntime::stop_effector(
    const std::uint64_t effector_asset_id, const std::uint64_t now_us) {
    std::lock_guard lifecycle_lock(asset_lifecycle_mutex_);
    const auto asset = registry_.asset(effector_asset_id, now_us);
    if (!asset || asset->role != AssetRole::effector)
        return DispatchError::command_rejected;
    return dispatch_safety_command(*asset, AttackAction::stop, 1, now_us);
}

AttackDispatchResult ServerRuntime::emergency_stop_effector(
    const std::uint64_t effector_asset_id, const std::uint64_t now_us) {
    std::lock_guard lifecycle_lock(asset_lifecycle_mutex_);
    const auto asset = registry_.asset(effector_asset_id, now_us);
    if (!asset || asset->role != AssetRole::effector)
        return DispatchError::command_rejected;
    return dispatch_safety_command(
        *asset, AttackAction::emergency_stop,
        config_.emergency_stop_repetitions, now_us);
}

AttackDispatchResult ServerRuntime::dispatch_safety_command(
    const AssetSnapshot& asset, const AttackAction action,
    const std::uint32_t repetitions, const std::uint64_t now_us) {
    if ((action != AttackAction::stop && action != AttackAction::emergency_stop) ||
        repetitions == 0) return DispatchError::command_rejected;
    const auto result = attack_commands_.create(action, 0, 0, now_us, {});
    if (!std::holds_alternative<AttackCommand>(result)) return DispatchError::command_rejected;
    auto command = std::get<AttackCommand>(result);
    command.header.asset_id = asset.asset_id;
    command.header.session_id = asset.session_id;
    effector_identity_.assign(command);
    const auto encoded = protobuf::encode(Envelope{command});
    if (action == AttackAction::stop) {
        if (command_tracker_.track({asset.asset_id, asset.session_id,
                                    command.command_id, encoded,
                                    asset.command_endpoint, now_us,
                                    command.valid_until_us}) !=
            CommandTrackResult::tracked)
            return DispatchError::command_rejected;
    }
    bool uncertain{};
    for (std::uint32_t attempt = 0; attempt < repetitions; ++attempt) {
        const CommandKey key{asset.asset_id, asset.session_id, command.command_id};
        const bool sent = action == AttackAction::stop
            ? send_tracked(encoded, asset.command_endpoint, key, now_us)
            : send_safety(encoded, asset.command_endpoint, key, now_us);
        uncertain |= !sent;
    }
    if (uncertain) return DispatchError::delivery_uncertain;
    return command;
}

AssignmentResult ServerRuntime::unassign(const std::uint64_t track_id) {
    std::lock_guard lifecycle_lock(asset_lifecycle_mutex_);
    const auto result = assignments_.unassign(track_id);
    if (result == AssignmentResult::completed) {
    }
    emit_event(config_.event_sink, {"unassignment_decision", 0, 0, 0, 0,
        "track=" + std::to_string(track_id) + " result=" + std::to_string(static_cast<int>(result))});
    return result;
}

EmergencyStopResult ServerRuntime::emergency_stop_all(
    const std::uint64_t now_us) {
    std::lock_guard lifecycle_lock(asset_lifecycle_mutex_);
    EmergencyStopResult result;
    if (now_us == 0 ||
        now_us > std::numeric_limits<std::uint64_t>::max() -
                     config_.attack_commands.command_validity_us)
        return result;
    const auto assets = registry_.known_assets(AssetRole::effector, now_us);
    for (const auto& asset : assets) {
        const auto dispatched = dispatch_safety_command(
            asset, AttackAction::emergency_stop,
            config_.emergency_stop_repetitions, now_us);
        if (std::holds_alternative<AttackCommand>(dispatched)) {
            ++result.assets;
            result.datagrams += config_.emergency_stop_repetitions;
        }
    }
    return result;
}

CommandRetryResult ServerRuntime::retry_unacknowledged(const std::uint64_t now_us) {
    std::lock_guard lifecycle_lock(asset_lifecycle_mutex_);
    const auto legacy = legacy_commands_.poll(now_us);
    CommandRetryResult result{0, legacy.exhausted};
    for (const auto& retry : legacy.transmissions) {
        if (send_safety(retry.datagram, retry.endpoint,
                        {0, 0, retry.command_id}, now_us)) ++result.resent;
        else ++result.send_failed;
    }
    const auto dynamic = command_tracker_.poll(now_us);
    for (const auto& retry : dynamic.transmissions) {
        const auto pending = command_tracker_.pending(retry.key);
        if (!pending || pending->state != PendingCommandState::awaiting_delivery) continue;
        if (send_tracked(retry.datagram, retry.endpoint, retry.key, now_us)) ++result.resent;
        else ++result.send_failed;
    }
    for (const auto& outcome : dynamic.finalized) {
        if (outcome.state == CommandTerminalState::delivery_exhausted)
            ++result.exhausted;
    }
    return result;
}

std::vector<PendingCommandSnapshot> ServerRuntime::pending_commands() const {
    return command_tracker_.pending_commands();
}

std::size_t ServerRuntime::pending_command_count() const {
    return legacy_commands_.pending_count() + command_tracker_.pending_count();
}

void ServerRuntime::send_heartbeats(
    const std::uint64_t now_us, const std::uint64_t uptime_ms) {
    heartbeats_.publish(registry_.assets(now_us), config_.observation_endpoint,
                        config_.effector_endpoint, now_us, uptime_ms);
}

bool ServerRuntime::pose_resynchronization_required(const ComponentId source) const {
    return connections_.pose_resynchronization_required(source);
}

ConnectionState ServerRuntime::connection_state(
    const ComponentId source, const std::uint64_t now_us) {
    return connections_.state(source, now_us);
}

std::vector<TargetCoordinate> ServerRuntime::targets(const std::uint64_t now_us) const {
    return state_.targets(now_us);
}

std::vector<AssetSnapshot> ServerRuntime::assets(const std::uint64_t now_us) {
    return registry_.assets(now_us);
}

std::vector<TrackSnapshot> ServerRuntime::tracks(const std::uint64_t now_us) {
    return tracks_.tracks(now_us);
}

AssignmentDecision ServerRuntime::assign(
    const std::uint64_t track_id, const std::uint64_t now_us) {
    std::lock_guard lifecycle_lock(asset_lifecycle_mutex_);
    const auto decision = assignment_coordinator_.assign(track_id, now_us);
    if (decision.assignment) emit_event(config_.event_sink, {"assignment_decision", now_us,
        decision.assignment->effector_asset_id, decision.assignment->effector_session_id, 0,
        "track=" + std::to_string(track_id) + " result=" + std::to_string(static_cast<int>(decision.result))});
    return decision;
}

AssignmentDecision ServerRuntime::assign(
    const std::uint64_t track_id, const std::uint64_t effector_asset_id,
    const std::uint64_t now_us) {
    std::lock_guard lifecycle_lock(asset_lifecycle_mutex_);
    const auto decision = assignment_coordinator_.assign(track_id, now_us, effector_asset_id);
    if (decision.assignment) emit_event(config_.event_sink, {"assignment_decision", now_us,
        decision.assignment->effector_asset_id, decision.assignment->effector_session_id, 0,
        "track=" + std::to_string(track_id) + " result=" + std::to_string(static_cast<int>(decision.result))});
    return decision;
}

std::optional<AssetAssignment> ServerRuntime::assignment(
    const std::uint64_t track_id) const {
    return assignments_.assignment(track_id);
}

std::vector<AssetAssignment> ServerRuntime::assignments() const {
    return assignments_.assignments();
}

std::vector<CommandOutcome> ServerRuntime::command_outcomes() const {
    return command_tracker_.outcomes();
}

std::vector<InboundRejection> ServerRuntime::inbound_rejections() const {
    return rejection_log_.snapshot();
}

std::optional<ObservationStatus> ServerRuntime::observation_status() const { return telemetry_.observation_status(); }
std::optional<EffectorStatus> ServerRuntime::effector_status() const { return telemetry_.effector_status(); }
std::optional<CommandAck> ServerRuntime::acknowledgement(ComponentId source, std::uint32_t command_id) const {
    return telemetry_.acknowledgement(source, command_id);
}
std::vector<ErrorReport> ServerRuntime::errors() const { return telemetry_.errors(); }

std::optional<DispatchError> ServerRuntime::connection_error(
    const ComponentId source, const std::uint64_t now_us) {
    if (connections_.state(source, now_us) != ConnectionState::connected)
        return DispatchError::connection_unavailable;
    if (connections_.pose_resynchronization_required(source))
        return DispatchError::pose_resynchronization_required;
    return std::nullopt;
}

template <typename Message>
void ServerRuntime::dispatch(const Message& message, const Endpoint& endpoint) {
    const auto encoded = protobuf::encode(Envelope{message});
    sender_(encoded, endpoint);
}

template <typename Message>
bool ServerRuntime::dispatch_tracked(
    const Message& message, const Endpoint& endpoint,
    const ComponentId acknowledgement_source, const std::uint64_t now_us) {
    auto encoded = protobuf::encode(Envelope{message});
    legacy_commands_.track(acknowledgement_source, message.command_id,
                           encoded, endpoint, now_us);
    return send_safety(encoded, endpoint, {0, 0, message.command_id}, now_us);
}

void ServerRuntime::audit_inbound(const Envelope& envelope, InboundResult result,
                                  std::uint64_t received_at_us) const {
    if (!config_.event_sink) return;
    std::visit([&](const auto& message) {
        if constexpr (requires { message.header; }) {
            std::uint32_t command_id{};
            if constexpr (requires { message.command_id; }) command_id = message.command_id;
            std::string detail = "kind=" + std::to_string(static_cast<int>(message_kind(envelope))) +
                                 " result=" + std::to_string(static_cast<int>(result));
            if constexpr (requires { message.result; })
                detail += " ack=" + std::to_string(static_cast<int>(message.result));
            emit_event(config_.event_sink, {result == InboundResult::accepted ? "inbound_accepted" : "inbound_rejected",
                received_at_us, message.header.asset_id, message.header.session_id, command_id,
                detail, static_cast<std::uint32_t>(message.header.source_id)});
        }
    }, envelope.payload);
}

bool ServerRuntime::send_tracked(std::span<const std::byte> bytes, const Endpoint& endpoint,
                                const CommandKey& key, std::uint64_t now_us) noexcept {
    audit_outbound(bytes, endpoint, now_us);
    bool succeeded{};
    try { sender_(bytes, endpoint); succeeded = true; } catch (...) {}
    command_tracker_.record_send(key, succeeded, now_us);
    return succeeded;
}
bool ServerRuntime::send_safety(std::span<const std::byte> bytes, const Endpoint& endpoint,
                               const CommandKey& key, std::uint64_t now_us) noexcept {
    audit_outbound(bytes, endpoint, now_us);
    bool succeeded{};
    try { sender_(bytes, endpoint); succeeded = true; } catch (...) {}
    emit_event(config_.event_sink, {succeeded ? "datagram_send_succeeded" : "datagram_delivery_uncertain",
        now_us, key.asset_id, key.session_id, key.command_id, endpoint.address + ":" + std::to_string(endpoint.port)});
    return succeeded;
}

void ServerRuntime::audit_outbound(std::span<const std::byte> bytes, const Endpoint& endpoint,
                                   std::uint64_t now_us) const noexcept {
    if (!config_.event_sink) return;
    try {
        const auto decoded = protobuf::decode(bytes);
        const auto* envelope = std::get_if<Envelope>(&decoded);
        if (!envelope) return;
        std::visit([&](const auto& message) {
            if constexpr (requires { message.command_id; }) {
                std::string detail = "kind=" + std::to_string(static_cast<int>(message_kind(*envelope))) +
                    " endpoint=" + endpoint.address + ":" + std::to_string(endpoint.port);
                if constexpr (requires { message.command_type; })
                    detail += " command_type=" + std::to_string(static_cast<int>(message.command_type));
                if constexpr (requires { message.action; })
                    detail += " action=" + std::to_string(static_cast<int>(message.action));
                if constexpr (requires { message.target_id; })
                    detail += " target=" + std::to_string(message.target_id);
                emit_event(config_.event_sink, {"command_send_attempt", now_us,
                    message.header.asset_id, message.header.session_id, message.command_id,
                    detail, static_cast<std::uint32_t>(message.header.destination_id)});
            }
        }, envelope->payload);
    } catch (...) {}
}
} // namespace c2
