#pragma once
#include "c2/runtime_types.hpp"
#include "c2/inbound_rejection_log.hpp"
#include "c2/track_update_publisher.hpp"
#include "c2/asset_assignment.hpp"
#include "c2/attack_command_service.hpp"
#include "c2/connection_monitor.hpp"
#include "c2/command_tracker.hpp"
#include "c2/legacy_command_tracker.hpp"
#include "c2/state_store.hpp"
#include "c2/telemetry_store.hpp"

namespace c2 {
class LegacyMessageRouter final {
public:
    LegacyMessageRouter(StateStore& state, TelemetryStore& telemetry,
        ConnectionMonitor& connections, AttackCommandService& attack_commands,
        LegacyCommandTracker& commands)
        : state_(state), telemetry_(telemetry), connections_(connections),
          attack_commands_(attack_commands), commands_(commands) {}
    [[nodiscard]] InboundResult receive(const Envelope& envelope, std::uint64_t received_at_us);
private:
    StateStore& state_;
    TelemetryStore& telemetry_;
    ConnectionMonitor& connections_;
    AttackCommandService& attack_commands_;
    LegacyCommandTracker& commands_;
};

class AssetMessageRouter final {
public:
    using SafetySender = std::function<AttackDispatchResult(
        const AssetSnapshot&, AttackAction, std::uint32_t, std::uint64_t)>;
    AssetMessageRouter(AssetRegistry& registry, TrackStore& tracks,
        CommandTracker& commands, AssetAssignmentService& assignments,
        LegacyMessageRouter& legacy, TrackUpdatePublisher& track_updates,
        InboundRejectionLog& rejections, std::mutex& lifecycle_mutex,
        std::uint32_t emergency_stop_repetitions, SafetySender safety_sender)
        : registry_(registry), tracks_(tracks), command_tracker_(commands),
          assignments_(assignments), legacy_(legacy), track_updates_(track_updates),
          rejections_(rejections), asset_lifecycle_mutex_(lifecycle_mutex),
          emergency_stop_repetitions_(emergency_stop_repetitions),
          safety_sender_(std::move(safety_sender)) {}
    [[nodiscard]] InboundResult receive(const Envelope& envelope, const Endpoint& source,
                                        std::uint64_t received_at_us);
private:
    AssetRegistry& registry_;
    TrackStore& tracks_;
    CommandTracker& command_tracker_;
    AssetAssignmentService& assignments_;
    LegacyMessageRouter& legacy_;
    TrackUpdatePublisher& track_updates_;
    InboundRejectionLog& rejections_;
    std::mutex& asset_lifecycle_mutex_;
    std::uint32_t emergency_stop_repetitions_;
    SafetySender safety_sender_;
};
} // namespace c2
