#pragma once
#include "control_center.hpp"
#include <ostream>

namespace c2 {
// Demo points are already in PROJECT_FRAME; real sensor processing must transform them.
class DemoLidarProcessor final : public ILidarProcessor {
public:
    std::vector<Observation> process(const LidarFrame& frame) override {
        std::vector<Observation> result;
        std::uint64_t id = 1;
        for (auto p : frame.points) result.push_back({frame.observer_id, id++, p, frame.measured_us});
        return result;
    }
};
// Deliberately no real pointing algorithm: the responsible team supplies this strategy.
class DemoPointingSolver final : public IPointingSolver {
public:
    std::optional<Angles> solve(const Asset&, const Observation&) const override {
        return Angles{15, 5};
    }
};
class ConsoleTransport final : public IAssetTransport {
    std::ostream& out_;
public:
    explicit ConsoleTransport(std::ostream& out) : out_(out) {}
    bool send_point(const PointCommand& c) override {
        out_ << "SIMULATED point asset=" << c.asset_id << " track=" << c.track_id
             << " pan=" << c.angles.pan_deg << " tilt=" << c.angles.tilt_deg << '\n';
        return true;
    }
    bool send_stop(std::uint64_t id) override {
        out_ << "SIMULATED stop asset=" << id << '\n'; return true;
    }
};
}
