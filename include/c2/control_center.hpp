#pragma once
#include <cmath>
#include <cstdint>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>
#include <deque>
#include <limits>
#include <utility>
#include <cstddef>

namespace c2 {
// Internal contracts only: these types do not define the network wire format.
struct Position { double x{}, y{}, z{}; };
inline bool finite(Position p) {
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
}
struct Observation {
    std::uint64_t observer_id{}, detection_id{};
    Position project_position;
    std::uint64_t measured_us{};
};
struct LidarFrame {
    std::uint64_t observer_id{}, measured_us{};
    std::vector<Position> points;
};
struct Asset {
    std::uint64_t id{};
    Position project_position;
    double azimuth_deg{};
    bool connected{};
    std::uint64_t last_seen_us{};
};
struct Angles { double pan_deg{}, tilt_deg{}; };
struct PointCommand { std::uint64_t asset_id{}, track_id{}; Angles angles; };
struct Limits {
    std::size_t max_assets{64}, max_tracks{256}, max_history{128};
    std::uint64_t target_validity_us{2'000'000}, heartbeat_timeout_us{3'000'000};
};
struct Event {
    std::uint64_t sequence{}, asset_id{}, track_id{};
    std::string action, result;
};

struct ILidarProcessor {
    virtual ~ILidarProcessor() = default;
    virtual std::vector<Observation> process(const LidarFrame&) = 0;
};
struct IPointingSolver {
    virtual ~IPointingSolver() = default;
    virtual std::optional<Angles> solve(const Asset&, const Observation&) const = 0;
};
struct IAssetTransport {
    virtual ~IAssetTransport() = default;
    // true means queued locally, not acknowledged or executed by hardware.
    virtual bool send_point(const PointCommand&) = 0;
    virtual bool send_stop(std::uint64_t asset_id) = 0;
};

class ControlCenter {
    ILidarProcessor& lidar_;
    const IPointingSolver& solver_;
    IAssetTransport& transport_;
    std::map<std::uint64_t, Asset> assets_;
    std::map<std::uint64_t, Observation> tracks_;
    std::map<std::pair<std::uint64_t, std::uint64_t>, std::uint64_t> identities_;
    std::uint64_t next_track_{1};
    Limits limits_;
    std::map<std::uint64_t, std::uint64_t> assignments_; // asset -> track
    std::deque<Event> events_;
    std::uint64_t next_event_{1};
    std::string last_result_;
    bool record(std::string action, std::string result,
                std::uint64_t asset = 0, std::uint64_t track = 0, bool success = false) {
        last_result_ = result;
        if (events_.size() == limits_.max_history) events_.pop_front();
        events_.push_back({next_event_++, asset, track, std::move(action), std::move(result)});
        return success;
    }
    bool fresh(std::uint64_t measured, std::uint64_t now, std::uint64_t validity) const {
        return measured <= now && now - measured <= validity;
    }
public:
    ControlCenter(ILidarProcessor& lidar, const IPointingSolver& solver,
                  IAssetTransport& transport, Limits limits = {})
        : lidar_(lidar), solver_(solver), transport_(transport), limits_(limits) {
        if (!limits.max_assets || !limits.max_tracks || !limits.max_history ||
            !limits.target_validity_us || !limits.heartbeat_timeout_us)
            throw std::invalid_argument("all limits must be positive");
    }
    void register_asset(Asset asset) {
        if (!asset.id || !finite(asset.project_position) || !std::isfinite(asset.azimuth_deg))
            throw std::invalid_argument("invalid asset");
        if (!assets_.contains(asset.id) && assets_.size() >= limits_.max_assets)
            throw std::length_error("asset capacity reached");
        // Re-registration never silently preserves an old assignment.
        assignments_.erase(asset.id);
        assets_[asset.id] = asset;
        record("register", "registered", asset.id, 0, true);
    }
    bool heartbeat(std::uint64_t asset_id, std::uint64_t measured_us, std::uint64_t now_us) {
        auto it = assets_.find(asset_id);
        if (it == assets_.end()) return record("heartbeat", "unknown asset", asset_id);
        if (!fresh(measured_us, now_us, limits_.heartbeat_timeout_us) ||
            measured_us <= it->second.last_seen_us)
            return record("heartbeat", "stale or future heartbeat", asset_id);
        it->second.last_seen_us = measured_us;
        it->second.connected = true;
        return record("heartbeat", "connected", asset_id, 0, true);
    }
    void tick(std::uint64_t now_us) {
        for (auto& [id, asset] : assets_) {
            if (asset.connected && !fresh(asset.last_seen_us, now_us, limits_.heartbeat_timeout_us)) {
                asset.connected = false;
                assignments_.erase(id);
                record("connection", "heartbeat expired", id);
            }
        }
        for (auto it = tracks_.begin(); it != tracks_.end();) {
            if (fresh(it->second.measured_us, now_us, limits_.target_validity_us)) { ++it; continue; }
            const auto track_id = it->first;
            identities_.erase({it->second.observer_id, it->second.detection_id});
            for (auto a = assignments_.begin(); a != assignments_.end();) {
                if (a->second == track_id) a = assignments_.erase(a); else ++a;
            }
            it = tracks_.erase(it);
            record("track", "expired", 0, track_id);
        }
    }
    void ingest(const LidarFrame& frame, std::uint64_t now_us) {
        tick(now_us);
        for (auto observation : lidar_.process(frame)) {
            if (!observation.observer_id || !observation.detection_id ||
                !finite(observation.project_position) || observation.measured_us > now_us ||
                now_us - observation.measured_us > limits_.target_validity_us) {
                record("observation", "invalid or expired observation"); continue;
            }
            if (observation.observer_id != frame.observer_id || observation.measured_us != frame.measured_us) {
                record("observation", "processor changed source or timestamp"); continue;
            }
            auto key = std::pair{observation.observer_id, observation.detection_id};
            if (!identities_.contains(key) && (tracks_.size() >= limits_.max_tracks ||
                next_track_ == std::numeric_limits<std::uint64_t>::max())) {
                record("observation", "track capacity reached"); continue;
            }
            auto [it, inserted] = identities_.try_emplace(key, next_track_);
            if (inserted) ++next_track_;
            auto old = tracks_.find(it->second);
            if (old != tracks_.end() && old->second.measured_us >= observation.measured_us) {
                record("observation", "duplicate or reordered observation", 0, it->second); continue;
            }
            tracks_[it->second] = observation;
            record("observation", "updated", 0, it->second, true);
        }
    }
    bool point(std::uint64_t asset_id, std::uint64_t track_id, std::uint64_t now_us) {
        tick(now_us);
        auto a = assets_.find(asset_id);
        auto t = tracks_.find(track_id);
        if (a == assets_.end()) return record("point", "unknown asset", asset_id, track_id);
        if (t == tracks_.end()) return record("point", "unknown or expired track", asset_id, track_id);
        if (!a->second.connected) return record("point", "asset disconnected", asset_id, track_id);
        if (auto assignment = assignments_.find(asset_id);
            assignment != assignments_.end() && assignment->second != track_id)
            return record("point", "asset assigned to another track", asset_id, track_id);
        auto angles = solver_.solve(a->second, t->second);
        if (!angles || !std::isfinite(angles->pan_deg) || !std::isfinite(angles->tilt_deg))
            return record("point", "solver rejected target", asset_id, track_id);
        bool queued = transport_.send_point({asset_id, track_id, *angles});
        return record("point", queued ? "queued (not ACK)" : "transport rejected", asset_id, track_id, queued);
    }
    bool assign(std::uint64_t asset_id, std::uint64_t track_id, std::uint64_t now_us) {
        tick(now_us);
        for (const auto& [id, track] : assignments_) {
            if (track == track_id && id != asset_id)
                return record("assign", "track already assigned", asset_id, track_id);
        }
        if (!point(asset_id, track_id, now_us)) return false;
        assignments_[asset_id] = track_id;
        return record("assign", "assigned; point queued (not ACK)", asset_id, track_id, true);
    }
    bool stop(std::uint64_t asset_id) {
        if (!assets_.contains(asset_id)) return record("stop", "unknown asset", asset_id);
        assignments_.erase(asset_id);
        bool queued = transport_.send_stop(asset_id);
        return record("stop", queued ? "queued (not ACK)" : "transport rejected; assignment cleared",
                      asset_id, 0, queued);
    }
    const auto& assets() const { return assets_; }
    const auto& tracks() const { return tracks_; }
    const auto& assignments() const { return assignments_; }
    const auto& events() const { return events_; }
    const auto& last_result() const { return last_result_; }
};
}
