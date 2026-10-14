#pragma once
#include "c2/track_update_publisher.hpp"

namespace c2 {
class HeartbeatPublisher final {
public:
    explicit HeartbeatPublisher(DatagramSender sender) : sender_(std::move(sender)) {}
    void publish(const std::vector<AssetSnapshot>& registered,
                 const Endpoint& observation_endpoint, const Endpoint& effector_endpoint,
                 std::uint64_t now_us, std::uint64_t uptime_ms) {
    std::lock_guard lock(heartbeat_mutex_);
    if (!registered.empty()) {
        for (const auto& asset : registered) {
            auto& sequence = asset.role == AssetRole::observation
                ? next_observation_heartbeat_sequence_
                : next_effector_heartbeat_sequence_;
            Heartbeat heartbeat{
                {protocol_version, sequence, now_us,
                 ComponentId::command_and_control,
                 asset.role == AssetRole::observation
                     ? ComponentId::observation_asset
                     : ComponentId::effector_asset,
                 asset.asset_id, asset.session_id},
                AssetOperatingState::operating, uptime_ms, now_us};
            send(heartbeat, asset.command_endpoint);
            sequence = CommandIdentity::next(sequence);
        }
        return;
    }
    Heartbeat observation{
        {protocol_version, next_observation_heartbeat_sequence_, now_us,
         ComponentId::command_and_control, ComponentId::observation_asset},
        AssetOperatingState::operating, uptime_ms, now_us};
    Heartbeat effector{
        {protocol_version, next_effector_heartbeat_sequence_, now_us,
         ComponentId::command_and_control, ComponentId::effector_asset},
        AssetOperatingState::operating, uptime_ms, now_us};
    send(observation, observation_endpoint);
    send(effector, effector_endpoint);
    next_observation_heartbeat_sequence_ =
        CommandIdentity::next(next_observation_heartbeat_sequence_);
    next_effector_heartbeat_sequence_ =
        CommandIdentity::next(next_effector_heartbeat_sequence_);
}
private:
    void send(const Heartbeat& heartbeat, const Endpoint& endpoint) {
        const auto bytes = protobuf::encode(Envelope{heartbeat});
        sender_(bytes, endpoint);
    }
    DatagramSender sender_;
    std::mutex heartbeat_mutex_;
    std::uint32_t next_observation_heartbeat_sequence_{1};
    std::uint32_t next_effector_heartbeat_sequence_{1};
};
} // namespace c2
