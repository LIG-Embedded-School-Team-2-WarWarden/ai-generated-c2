#include "c2/inbound_message_router.hpp"
#include <type_traits>

namespace c2 {
InboundResult AssetMessageRouter::receive(const Envelope& envelope, const Endpoint& source,
                                         const std::uint64_t received_at_us) {
    const auto kind = message_kind(envelope);
    const auto& payload = envelope.payload;
    return std::visit(
        [&](const auto& message) -> InboundResult {
            using T = std::decay_t<decltype(message)>;
            if constexpr (std::is_same_v<T, AssetRegistration>) {
                std::lock_guard lifecycle_lock(asset_lifecycle_mutex_);
                const auto previous = registry_.asset(
                    message.header.asset_id, received_at_us);
                const auto result = registry_.register_asset(
                    message, source, received_at_us);
                if (result == AssetRegistryResult::session_replaced && previous &&
                    previous->session_id != message.header.session_id) {
                    (void)command_tracker_.end_session(
                        previous->asset_id, previous->session_id, received_at_us);
                    if (previous->role == AssetRole::effector)
                        (void)assignments_.mark_unavailable(
                            previous->asset_id, previous->session_id,
                            received_at_us);
                    if (previous->role == AssetRole::effector)
                        (void)safety_sender_(
                            *previous, AttackAction::emergency_stop,
                            emergency_stop_repetitions_, received_at_us);
                }
                return result == AssetRegistryResult::registered ||
                               result == AssetRegistryResult::refreshed ||
                               result == AssetRegistryResult::session_replaced
                           ? InboundResult::accepted
                           : (rejections_.record(
                                  InboundRejectionCategory::registration,
                                  kind, &message.header, result, received_at_us),
                              InboundResult::rejected);
            } else if constexpr (std::is_same_v<T, AssetUnregister>) {
                std::lock_guard lifecycle_lock(asset_lifecycle_mutex_);
                const auto result = registry_.unregister_asset(
                    message, source, received_at_us);
                if (result != AssetRegistryResult::unregistered) {
                    rejections_.record(
                        InboundRejectionCategory::authentication,
                        kind, &message.header, result, received_at_us);
                    return InboundResult::rejected;
                }
                (void)command_tracker_.end_session(
                    message.header.asset_id, message.header.session_id,
                    received_at_us);
                (void)assignments_.mark_unavailable(
                    message.header.asset_id, message.header.session_id,
                    received_at_us);
                return InboundResult::accepted;
            } else if constexpr (std::is_same_v<T, Heartbeat>) {
                const auto result = registry_.observe_heartbeat(
                    message, source, received_at_us);
                if (result != AssetRegistryResult::stored &&
                    result != AssetRegistryResult::duplicate) {
                    rejections_.record(
                        InboundRejectionCategory::authentication,
                        kind, &message.header, result, received_at_us);
                    return InboundResult::rejected;
                }
            } else if constexpr (std::is_same_v<T, AssetPose>) {
                const auto result = registry_.update_pose(
                    message, source, received_at_us);
                if (result != AssetRegistryResult::stored &&
                    result != AssetRegistryResult::duplicate) {
                    rejections_.record(
                        InboundRejectionCategory::authentication,
                        kind, &message.header, result, received_at_us);
                    return InboundResult::rejected;
                }
            } else if constexpr (std::is_same_v<T, TargetCoordinate>) {
                std::lock_guard lifecycle_lock(asset_lifecycle_mutex_);
                const auto authenticated = registry_.authenticate(
                    message.header, source, received_at_us);
                if (authenticated != AssetRegistryResult::stored) {
                    rejections_.record(
                        InboundRejectionCategory::authentication,
                        kind, &message.header, authenticated, received_at_us);
                    return InboundResult::rejected;
                }
                const auto result = tracks_.update(message, received_at_us);
                if (result.result == TrackUpdateResult::stored ||
                    result.result == TrackUpdateResult::duplicate) {
                    if (result.result == TrackUpdateResult::stored) {
                        const auto assignment = assignments_.assignment(result.track_id);
                        const auto track = tracks_.track(result.track_id, received_at_us);
                        const auto asset = assignment
                            ? registry_.asset(assignment->effector_asset_id, received_at_us)
                            : std::nullopt;
                        if (assignment && track && asset &&
                            assignment->state == AssignmentResult::assigned &&
                            asset->session_id == assignment->effector_session_id)
                            track_updates_.publish(*track, *asset, received_at_us);
                    }
                    return InboundResult::accepted;
                }
                rejections_.record(
                    InboundRejectionCategory::state_update,
                    kind, &message.header, AssetRegistryResult::invalid,
                    received_at_us);
                return InboundResult::rejected;
            } else if constexpr (std::is_same_v<T, CommandAck>) {
                const auto authenticated = registry_.authenticate(
                    message.header, source, received_at_us);
                if (authenticated != AssetRegistryResult::stored) {
                    rejections_.record(
                        InboundRejectionCategory::authentication,
                        kind, &message.header, authenticated, received_at_us);
                    return InboundResult::rejected;
                }
                const auto result = command_tracker_.observe(
                    message, received_at_us);
                if (result == AckUpdateResult::progress ||
                    result == AckUpdateResult::terminal ||
                    result == AckUpdateResult::duplicate)
                    return InboundResult::accepted;
                rejections_.record(
                    InboundRejectionCategory::state_update,
                    kind, &message.header, AssetRegistryResult::invalid,
                    received_at_us);
                return InboundResult::rejected;
            } else if constexpr (std::is_same_v<T, EffectorStatus>) {
                const auto result = registry_.update_effector_status(
                    message, source, received_at_us);
                if (result == AssetRegistryResult::stored ||
                    result == AssetRegistryResult::duplicate)
                    return InboundResult::accepted;
                rejections_.record(
                    InboundRejectionCategory::authentication,
                    kind, &message.header, result, received_at_us);
                return InboundResult::rejected;
            } else if constexpr (std::is_same_v<T, ObservationStatus> ||
                                 std::is_same_v<T, ErrorReport>) {
                const auto authenticated = registry_.authenticate(
                    message.header, source, received_at_us);
                if (authenticated != AssetRegistryResult::stored) {
                    rejections_.record(
                        InboundRejectionCategory::authentication,
                        kind, &message.header, authenticated, received_at_us);
                    return InboundResult::rejected;
                }
            } else {
                if constexpr (std::is_same_v<T, std::monostate>) {
                    rejections_.record(
                        InboundRejectionCategory::unsupported_message,
                        kind, nullptr, AssetRegistryResult::invalid,
                        received_at_us);
                } else {
                    rejections_.record(
                        InboundRejectionCategory::unsupported_message,
                        kind, &message.header, AssetRegistryResult::invalid,
                        received_at_us);
                }
                return InboundResult::unsupported_message;
            }
            return legacy_.receive(envelope, received_at_us);
        },
        payload);
}

InboundResult LegacyMessageRouter::receive(const Envelope& envelope,
                                          const std::uint64_t received_at_us) {
    const auto& payload = envelope.payload;
    return std::visit(
        [&](const auto& message) -> InboundResult {
            using T = std::decay_t<decltype(message)>;
            if constexpr (std::is_same_v<T, AssetPose>) {
                const auto result = state_.update(message);
                if (result == StoreUpdateResult::stored) {
                    (void)connections_.mark_pose_synchronized(
                        message.header.source_id, received_at_us);
                    return InboundResult::accepted;
                }
                return result == StoreUpdateResult::duplicate
                           ? InboundResult::accepted
                           : InboundResult::rejected;
            } else if constexpr (std::is_same_v<T, TargetCoordinate>) {
                const auto result = state_.update(message, received_at_us);
                return result == StoreUpdateResult::stored ||
                               result == StoreUpdateResult::duplicate
                           ? InboundResult::accepted
                           : InboundResult::rejected;
            } else if constexpr (std::is_same_v<T, Heartbeat>) {
                const auto result = connections_.observe(message, received_at_us);
                return result == HeartbeatUpdateResult::stored ||
                               result == HeartbeatUpdateResult::duplicate
                           ? InboundResult::accepted
                           : InboundResult::rejected;
            } else if constexpr (std::is_same_v<T, EffectorStatus>) {
                const auto telemetry_result = telemetry_.update(message);
                const auto safety_result = attack_commands_.update_status(message);
                return (telemetry_result == TelemetryUpdateResult::stored || telemetry_result == TelemetryUpdateResult::duplicate) &&
                               (safety_result == AttackStatusUpdateResult::stored || safety_result == AttackStatusUpdateResult::duplicate)
                           ? InboundResult::accepted
                           : InboundResult::rejected;
            } else if constexpr (std::is_same_v<T, CommandAck>) {
                const auto result = telemetry_.update(message);
                if (result == TelemetryUpdateResult::stored ||
                    result == TelemetryUpdateResult::duplicate) {
                    commands_.acknowledge(message);
                    return InboundResult::accepted;
                }
                return InboundResult::rejected;
            } else if constexpr (std::is_same_v<T, ObservationStatus> ||
                                 std::is_same_v<T, ErrorReport>) {
                const auto result = telemetry_.update(message);
                return result == TelemetryUpdateResult::stored ||
                               result == TelemetryUpdateResult::duplicate
                           ? InboundResult::accepted
                           : InboundResult::rejected;
            } else {
                return InboundResult::unsupported_message;
            }
        },
        payload);
}

} // namespace c2
