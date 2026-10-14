#pragma once

#include "c2/asset_registry.hpp"
#include "c2/runtime_types.hpp"
#include "c2/command_identity.hpp"
#include "c2/inbound_rejection_log.hpp"
#include "c2/track_update_publisher.hpp"
#include "c2/inbound_message_router.hpp"
#include "c2/heartbeat_publisher.hpp"
#include "c2/assignment_coordinator.hpp"
#include "c2/asset_assignment.hpp"
#include "c2/attack_command_service.hpp"
#include "c2/connection_monitor.hpp"
#include "c2/command_tracker.hpp"
#include "c2/legacy_command_tracker.hpp"
#include "c2/effector_command_service.hpp"
#include "c2/observation_command_service.hpp"
#include "c2/state_store.hpp"
#include "c2/telemetry_store.hpp"
#include "c2/track_store.hpp"
#include "c2/udp_transport.hpp"

#include <cstddef>
#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>
#include <span>
#include <variant>
#include <vector>

namespace c2 {
struct ServerRuntimeConfig {
    StateStoreConfig state;
    ConnectionMonitorConfig connections;
    ObservationCommandConfig observation_commands;
    EffectorCommandConfig effector_commands;
    AttackCommandConfig attack_commands;
    Endpoint observation_endpoint;
    Endpoint effector_endpoint;
    std::uint32_t emergency_stop_repetitions{3};
    std::uint64_t command_ack_timeout_us{200'000};
    std::uint32_t command_max_attempts{3};
    AssetRegistryConfig registry;
    TrackStoreConfig tracks;
    CommandTrackerConfig commands;
    AssetAssignmentConfig assignments;
    std::size_t maximum_error_history{128};
    std::size_t maximum_inbound_rejections{128};
    EventSink event_sink;
};

class ServerRuntime final {
public:
    using DatagramSender = c2::DatagramSender;

    ServerRuntime(ServerRuntimeConfig config, DatagramSender sender);
    [[nodiscard]] DevelopmentPoseDispatchResult set_development_pose(
        std::uint64_t asset_id, float x_m, float y_m, float z_m,
        float azimuth_deg, std::uint64_t now_us);

    [[nodiscard]] InboundResult ingest(
        std::span<const std::byte> datagram, std::uint64_t received_at_us);
    [[nodiscard]] InboundResult ingest(
        std::span<const std::byte> datagram, const Endpoint& source,
        std::uint64_t received_at_us);
    [[nodiscard]] ObservationDispatchResult command_observation(
        ObservationTurretCommandType type, float pan_deg, float tilt_deg,
        std::uint64_t now_us);
    [[nodiscard]] ObservationDispatchResult command_observation(
        std::uint64_t asset_id, ObservationTurretCommandType type,
        float pan_deg, float tilt_deg, std::uint64_t now_us);
    [[nodiscard]] EffectorDispatchResult point_effector(
        std::uint64_t target_id, std::uint64_t now_us);
    [[nodiscard]] AttackDispatchResult attack(
        AttackAction action, std::uint64_t target_id,
        std::uint32_t duration_ms, std::uint64_t now_us);
    [[nodiscard]] AttackDispatchResult stop_effector(
        std::uint64_t effector_asset_id, std::uint64_t now_us);
    [[nodiscard]] AttackDispatchResult emergency_stop_effector(
        std::uint64_t effector_asset_id, std::uint64_t now_us);
    [[nodiscard]] AssignmentResult unassign(std::uint64_t track_id);
    [[nodiscard]] EmergencyStopResult emergency_stop_all(std::uint64_t now_us);
    void send_heartbeats(std::uint64_t now_us, std::uint64_t uptime_ms);
    [[nodiscard]] CommandRetryResult retry_unacknowledged(std::uint64_t now_us);
    [[nodiscard]] std::size_t pending_command_count() const;
    [[nodiscard]] std::vector<PendingCommandSnapshot> pending_commands() const;
    [[nodiscard]] bool pose_resynchronization_required(ComponentId source) const;
    [[nodiscard]] ConnectionState connection_state(
        ComponentId source, std::uint64_t now_us);
    [[nodiscard]] std::vector<TargetCoordinate> targets(std::uint64_t now_us) const;
    [[nodiscard]] std::vector<AssetSnapshot> assets(std::uint64_t now_us);
    [[nodiscard]] std::vector<TrackSnapshot> tracks(std::uint64_t now_us);
    [[nodiscard]] AssignmentDecision assign(
        std::uint64_t track_id, std::uint64_t now_us);
    [[nodiscard]] AssignmentDecision assign(
        std::uint64_t track_id, std::uint64_t effector_asset_id,
        std::uint64_t now_us);
    [[nodiscard]] std::optional<AssetAssignment> assignment(
        std::uint64_t track_id) const;
    [[nodiscard]] std::vector<AssetAssignment> assignments() const;
    [[nodiscard]] std::vector<CommandOutcome> command_outcomes() const;
    [[nodiscard]] std::vector<InboundRejection> inbound_rejections() const;
    [[nodiscard]] std::optional<ObservationStatus> observation_status() const;
    [[nodiscard]] std::optional<EffectorStatus> effector_status() const;
    [[nodiscard]] std::optional<CommandAck> acknowledgement(
        ComponentId source, std::uint32_t command_id) const;
    [[nodiscard]] std::vector<ErrorReport> errors() const;

private:
    [[nodiscard]] std::optional<DispatchError> connection_error(
        ComponentId source, std::uint64_t now_us);
    template <typename Message>
    void dispatch(const Message& message, const Endpoint& endpoint);
    template <typename Message>
    bool dispatch_tracked(
        const Message& message, const Endpoint& endpoint,
        ComponentId acknowledgement_source, std::uint64_t now_us);
    [[nodiscard]] AttackDispatchResult dispatch_safety_command(
        const AssetSnapshot& asset, AttackAction action,
        std::uint32_t repetitions, std::uint64_t now_us);
    void audit_inbound(const Envelope& envelope, InboundResult result, std::uint64_t received_at_us) const;
    void audit_outbound(std::span<const std::byte> bytes, const Endpoint& endpoint,
                        std::uint64_t now_us) const noexcept;
    bool send_tracked(std::span<const std::byte> bytes, const Endpoint& endpoint,
                      const CommandKey& key, std::uint64_t now_us) noexcept;
    bool send_safety(std::span<const std::byte> bytes, const Endpoint& endpoint,
                     const CommandKey& key, std::uint64_t now_us) noexcept;
    ServerRuntimeConfig config_;
    DatagramSender sender_;
    AssetRegistry registry_;
    TrackStore tracks_;
    CommandTracker command_tracker_;
    AssetAssignmentService assignments_;
    StateStore state_;
    TelemetryStore telemetry_;
    ConnectionMonitor connections_;
    ObservationCommandService observation_commands_;
    EffectorCommandService effector_commands_;
    AttackCommandService attack_commands_;
    CommandIdentity effector_identity_;
    TrackUpdatePublisher track_updates_;
    HeartbeatPublisher heartbeats_;
    AssignmentCoordinator assignment_coordinator_;
    LegacyCommandTracker legacy_commands_;
    std::mutex asset_lifecycle_mutex_;
    InboundRejectionLog rejection_log_;
    LegacyMessageRouter legacy_inbound_;
    AssetMessageRouter asset_inbound_;
};
}  // namespace c2
