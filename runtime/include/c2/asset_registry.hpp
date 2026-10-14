#pragma once

#include "c2/protocol.hpp"
#include "c2/udp_transport.hpp"

#include <cstddef>
#include <cstdint>
#include <map>
#include <mutex>
#include <optional>
#include <vector>

namespace c2 {
struct AssetRegistryConfig {
    std::size_t maximum_assets{256};
    std::uint64_t heartbeat_timeout_us{3'000'000};
    std::uint64_t retired_retention_us{60'000'000};
    std::uint64_t status_timeout_us{1'000'000};
    std::uint64_t maximum_lease_duration_ms{60'000};
};

enum class AssetRegistryResult {
    registered,
    refreshed,
    session_replaced,
    stored,
    duplicate,
    stale,
    unregistered,
    invalid,
    not_registered,
    session_mismatch,
    endpoint_mismatch,
    role_mismatch,
    capacity_exceeded,
};

enum class AssetConnectionState {
    awaiting_heartbeat,
    connected,
    disconnected,
    lease_expired,
    unregistered,
};

struct AssetSnapshot {
    std::uint64_t asset_id{};
    std::uint64_t session_id{};
    AssetRole role{AssetRole::unspecified};
    std::uint64_t capabilities{};
    std::string software_version;
    std::string hardware_version;
    PanTiltLimits turret_limits;
    bool concurrent_tasks{};
    Endpoint command_endpoint;
    Endpoint source_endpoint;
    std::uint64_t registered_at_us{};
    std::uint64_t lease_expires_at_us{};
    std::uint64_t last_heartbeat_received_at_us{};
    AssetConnectionState connection_state{AssetConnectionState::awaiting_heartbeat};
    bool pose_synchronized{};
    std::optional<AssetPose> pose;
    std::optional<EffectorStatus> effector_status;
    bool status_current{};
};

class AssetRegistry final {
public:
    explicit AssetRegistry(AssetRegistryConfig config);

    [[nodiscard]] AssetRegistryResult register_asset(
        const AssetRegistration& registration,
        const Endpoint& source,
        std::uint64_t received_at_us);
    [[nodiscard]] AssetRegistryResult unregister_asset(
        const AssetUnregister& unregister_message,
        const Endpoint& source,
        std::uint64_t received_at_us);
    [[nodiscard]] AssetRegistryResult observe_heartbeat(
        const Heartbeat& heartbeat,
        const Endpoint& source,
        std::uint64_t received_at_us);
    [[nodiscard]] AssetRegistryResult update_pose(
        const AssetPose& pose,
        const Endpoint& source,
        std::uint64_t received_at_us);
    [[nodiscard]] AssetRegistryResult update_effector_status(
        const EffectorStatus& status,
        const Endpoint& source,
        std::uint64_t received_at_us);
    [[nodiscard]] AssetRegistryResult authenticate(
        const MessageHeader& header,
        const Endpoint& source,
        std::uint64_t received_at_us);

    [[nodiscard]] std::optional<AssetSnapshot> asset(
        std::uint64_t asset_id, std::uint64_t now_us) const;
    [[nodiscard]] std::optional<AssetConnectionState> connection_state(
        std::uint64_t asset_id, std::uint64_t now_us) const;
    [[nodiscard]] std::vector<AssetSnapshot> assets(
        AssetRole role, std::uint64_t now_us);
    [[nodiscard]] std::vector<AssetSnapshot> assets(std::uint64_t now_us);
    [[nodiscard]] std::vector<AssetSnapshot> known_assets(
        AssetRole role, std::uint64_t now_us);
    std::size_t expire(std::uint64_t now_us);
    std::size_t prune(std::uint64_t now_us);

private:
    struct Entry {
        AssetRegistration registration;
        Endpoint source_endpoint;
        Endpoint command_endpoint;
        std::uint64_t registered_at_us{};
        std::uint64_t lease_expires_at_us{};
        std::uint64_t last_heartbeat_received_at_us{};
        std::optional<Heartbeat> heartbeat;
        std::optional<AssetPose> pose;
        std::optional<EffectorStatus> effector_status;
        std::uint64_t status_received_at_us{};
        bool active{true};
        AssetConnectionState terminal_state{AssetConnectionState::unregistered};
        std::uint64_t retired_at_us{};
    };

    [[nodiscard]] AssetRegistryResult validate_message_locked(
        const MessageHeader& header, const Endpoint& source, const Entry*& entry) const;
    [[nodiscard]] AssetConnectionState state_locked(
        const Entry& entry, std::uint64_t now_us) const noexcept;
    [[nodiscard]] AssetSnapshot snapshot_locked(
        const Entry& entry, std::uint64_t now_us) const;
    void expire_locked(std::uint64_t now_us, std::size_t& count);

    AssetRegistryConfig config_;
    mutable std::mutex mutex_;
    std::map<std::uint64_t, Entry> entries_;
};
}  // namespace c2
