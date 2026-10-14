#pragma once
#include "c2/protocol.hpp"
#include "c2/protocol_validation.hpp"
#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>

namespace c2 {
namespace dummy_error {
inline constexpr std::uint32_t communication_timeout = 0x1001;
inline constexpr std::uint32_t out_of_range = 0x3003;
inline constexpr std::uint32_t expired_command = 0x4001;
inline constexpr std::uint32_t duplicate_command = 0x4002;
inline constexpr std::uint32_t invalid_command = 0x4003;
inline constexpr std::uint32_t invalid_state = 0x4004;
inline constexpr std::uint32_t not_aligned = 0x5001;
inline constexpr std::uint32_t not_armed = 0x5002;
inline constexpr std::uint32_t target_expired = 0x5003;
inline constexpr std::uint32_t invalid_target_state = 0x5004;
inline constexpr std::uint32_t track_mismatch = 0x5005;
}  // namespace dummy_error

struct ProcessedCommand {
    CommandAck acknowledgement;
    bool duplicate{};
    std::optional<ErrorReport> error_report;
};

class DummyObservationAsset final {
public:
    [[nodiscard]] ProcessedCommand handle(const DevelopmentPoseCommand& command, std::uint64_t now_us);
    DummyObservationAsset(
        AssetPose pose, ObservationTurretLimits limits,
        std::size_t maximum_cached_results = 1024);
    [[nodiscard]] ProcessedCommand handle(const ObservationTurretCommand& command, std::uint64_t now_us);
    [[nodiscard]] AssetPose asset_pose(std::uint64_t now_us);
    [[nodiscard]] ObservationStatus status(std::uint64_t now_us);
    [[nodiscard]] Heartbeat heartbeat(std::uint64_t now_us, std::uint64_t uptime_ms);
    [[nodiscard]] TargetCoordinate target(std::uint32_t detection_id, float x_m, float y_m, float z_m, float confidence, std::uint64_t now_us);
    [[nodiscard]] TargetCoordinate target(
        std::uint32_t detection_id, float x_m, float y_m, float z_m,
        float vx_mps, float vy_mps, float vz_mps, bool velocity_valid,
        float confidence, std::uint64_t now_us);
    [[nodiscard]] bool observe_control_heartbeat(
        const Heartbeat& heartbeat, std::uint64_t received_at_us);
    [[nodiscard]] std::optional<ErrorReport> check_watchdog(
        std::uint64_t now_us, std::uint64_t timeout_us);
private:
    MessageHeader header(std::uint64_t now_us);
    CommandAck ack(std::uint32_t command_id, CommandResult result, std::uint32_t error, std::uint64_t now_us);
    ErrorReport error(
        std::uint32_t error_code, ErrorSeverity severity,
        std::uint32_t command_id, std::string detail, std::uint64_t now_us);
    void remember(std::uint32_t command_id, const CommandAck& result);
    AssetPose pose_;
    std::uint64_t asset_id_{};
    std::uint64_t session_id_{};
    ObservationTurretLimits limits_;
    ObservationStatus status_;
    std::uint32_t sequence_{1};
    std::unordered_map<std::uint32_t, CommandAck> results_;
    std::deque<std::uint32_t> result_order_;
    std::size_t maximum_cached_results_;
    std::optional<std::uint32_t> latest_command_id_;
    std::uint64_t last_control_heartbeat_us_{};
    bool watchdog_tripped_{};
    mutable std::mutex mutex_;
};

class DummyEffectorAsset final {
public:
    [[nodiscard]] ProcessedCommand handle(const DevelopmentPoseCommand& command, std::uint64_t now_us);
    explicit DummyEffectorAsset(
        AssetPose pose,
        ObservationTurretLimits limits = {-180, 180, -90, 90},
        std::size_t maximum_cached_results = 1024);
    [[nodiscard]] ProcessedCommand handle(const EffectorTurretCommand& command, std::uint64_t now_us);
    [[nodiscard]] ProcessedCommand handle(const AttackCommand& command, std::uint64_t now_us);
    [[nodiscard]] bool handle(const TargetTrackUpdate& update, std::uint64_t now_us);
    [[nodiscard]] std::optional<ErrorReport> control_step(
        std::uint64_t now_us, std::uint64_t maximum_prediction_us = 2'000'000);
    [[nodiscard]] AssetPose asset_pose(std::uint64_t now_us);
    [[nodiscard]] EffectorStatus status(std::uint64_t now_us);
    [[nodiscard]] Heartbeat heartbeat(std::uint64_t now_us, std::uint64_t uptime_ms);
    [[nodiscard]] bool observe_control_heartbeat(
        const Heartbeat& heartbeat, std::uint64_t received_at_us);
    [[nodiscard]] std::optional<ErrorReport> check_watchdog(
        std::uint64_t now_us, std::uint64_t timeout_us);
private:
    MessageHeader header(std::uint64_t now_us);
    CommandAck ack(std::uint32_t command_id, CommandResult result, std::uint32_t error, std::uint64_t now_us);
    ErrorReport error(
        std::uint32_t error_code, ErrorSeverity severity,
        std::uint32_t command_id, std::string detail, std::uint64_t now_us);
    void remember(std::uint32_t command_id, const CommandAck& result);
    void advance(std::uint64_t now_us);
    void safe_stop(TrackingStopReason reason, std::uint32_t error_code);
    AssetPose pose_;
    std::uint64_t asset_id_{};
    std::uint64_t session_id_{};
    ObservationTurretLimits limits_;
    EffectorStatus status_;
    std::uint64_t attack_end_us_{};
    std::optional<TargetTrackUpdate> target_update_;
    std::uint32_t sequence_{1};
    std::unordered_map<std::uint32_t, CommandAck> results_;
    std::deque<std::uint32_t> result_order_;
    std::size_t maximum_cached_results_;
    std::optional<std::uint32_t> latest_command_id_;
    std::uint64_t last_control_heartbeat_us_{};
    bool watchdog_tripped_{};
    mutable std::mutex mutex_;
};
}  // namespace c2
