#pragma once
#include <cmath>
#include <cstdint>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

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
};
struct Angles { double pan_deg{}, tilt_deg{}; };
struct PointCommand { std::uint64_t asset_id{}, track_id{}; Angles angles; };

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
public:
    ControlCenter(ILidarProcessor& lidar, const IPointingSolver& solver,
                  IAssetTransport& transport)
        : lidar_(lidar), solver_(solver), transport_(transport) {}
    void register_asset(Asset asset) {
        if (!asset.id || !finite(asset.project_position) || !std::isfinite(asset.azimuth_deg))
            throw std::invalid_argument("invalid asset");
        assets_[asset.id] = asset;
    }
    void ingest(const LidarFrame& frame, std::uint64_t now_us) {
        for (auto observation : lidar_.process(frame)) {
            if (!observation.observer_id || !observation.detection_id ||
                !finite(observation.project_position) || observation.measured_us > now_us ||
                now_us - observation.measured_us > 2'000'000) continue;
            auto key = std::pair{observation.observer_id, observation.detection_id};
            auto [it, inserted] = identities_.try_emplace(key, next_track_);
            if (inserted) ++next_track_;
            auto old = tracks_.find(it->second);
            if (old != tracks_.end() && old->second.measured_us >= observation.measured_us) continue;
            tracks_[it->second] = observation;
        }
    }
    bool point(std::uint64_t asset_id, std::uint64_t track_id, std::uint64_t now_us) {
        auto a = assets_.find(asset_id);
        auto t = tracks_.find(track_id);
        if (a == assets_.end() || t == tracks_.end() || !a->second.connected ||
            t->second.measured_us > now_us || now_us - t->second.measured_us > 2'000'000) return false;
        auto angles = solver_.solve(a->second, t->second);
        if (!angles || !std::isfinite(angles->pan_deg) || !std::isfinite(angles->tilt_deg)) return false;
        return transport_.send_point({asset_id, track_id, *angles});
    }
    bool stop(std::uint64_t asset_id) {
        return assets_.contains(asset_id) && transport_.send_stop(asset_id);
    }
    const auto& assets() const { return assets_; }
    const auto& tracks() const { return tracks_; }
};
}
