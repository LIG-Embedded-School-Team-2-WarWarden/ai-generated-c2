#pragma once
#include "c2/asset_registry.hpp"
#include "c2/datagram_sender.hpp"
#include "c2/event_log.hpp"
#include "c2/track_store.hpp"
#include "c2/command_identity.hpp"
#include "c2/protobuf_codec.hpp"
#include "c2/protocol_validation.hpp"
#include <functional>
#include <span>
#include <utility>

namespace c2 {
class TrackUpdatePublisher final {
public:
    explicit TrackUpdatePublisher(DatagramSender sender, EventSink events = {})
        : sender_(std::move(sender)), events_(std::move(events)) {
        if (!sender_) throw std::invalid_argument("datagram sender must be set");
    }
    bool publish(const TrackSnapshot& track, const AssetSnapshot& asset, std::uint64_t now_us) {
        const auto& measurement = track.measurement;
        if (!measurement.velocity_valid || asset.role != AssetRole::effector ||
            asset.connection_state != AssetConnectionState::connected) return true;
        TargetTrackUpdate update{
            {protocol_version, 1, now_us, ComponentId::command_and_control,
             ComponentId::effector_asset, asset.asset_id, asset.session_id},
            track.track_id, CoordinateFrame::project_frame,
            measurement.x_m, measurement.y_m, measurement.z_m,
            measurement.vx_mps, measurement.vy_mps, measurement.vz_mps,
            true, measurement.measurement_time_us, track.expires_at_us,
            measurement.confidence, track.observation_asset_id, track.observation_session_id};
        {
            std::lock_guard lock(mutex_);
            update.header.sequence = sequence_;
            sequence_ = CommandIdentity::next(sequence_);
        }
        if (validate(update).valid()) {
            const auto bytes = protobuf::encode(Envelope{update});
            try { sender_(bytes, asset.command_endpoint); return true; }
            catch (...) {
                emit_event(events_, {"track_stream_send_failed", now_us, asset.asset_id,
                    asset.session_id, 0, std::to_string(track.track_id)});
                return false;
            }
        }
        return false;
    }
private:
    DatagramSender sender_;
    EventSink events_;
    std::mutex mutex_;
    std::uint32_t sequence_{1};
};
} // namespace c2
