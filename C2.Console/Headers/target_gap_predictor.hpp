#pragma once
#include "rdr_coordinates.hpp"

namespace c2::rdr {
// Short camera-tracking fallback for missing DE03 traffic. Sender predictions
// remain authoritative; only observed world positions establish velocity.
class TargetGapPredictor {
    struct History {
        BasePoint position{}, velocity{};
        double observed{}, received{};
        bool has_velocity{};
    };
    std::map<uint16_t,History> history_;
    uint64_t generation_{};
    BasePoint origin_{};
public:
    static constexpr double gap_delay_s=.2;
    static constexpr double horizon_s=1.5;
    void reset() { history_.clear(); }
    std::vector<CorrectedTarget> update(const State& state,
        const std::vector<CorrectedTarget>& filtered, BasePoint origin, double now) {
        if (generation_!=state.generation || origin.east!=origin_.east ||
            origin.north!=origin_.north || origin.up!=origin_.up) reset();
        generation_=state.generation;origin_=origin;
        auto result=filtered;
        if (!state.connected || !state.reporting || state.mode!=2 || !std::isfinite(now)) {
            reset();return result;
        }
        for(auto it=history_.begin();it!=history_.end();)
            if(!state.targets.contains(it->first)) it=history_.erase(it);else ++it;
        for(auto& point:result) {
            const auto& raw=state.targets.at(point.id);
            if(!point.valid || state.excluded.contains(point.id) || !std::isfinite(raw.seen) || now<raw.seen) {
                history_.erase(point.id);point.valid=false;continue;
            }
            auto it=history_.find(point.id);
            if(it!=history_.end() && raw.seen<it->second.received) {
                history_.erase(it);it=history_.end();
            }
            if(raw.track_state==1 && raw.detected==1 &&
                (it==history_.end() || raw.seen>it->second.received)) {
                History next{point.world,{},raw.seen,raw.seen,false};
                if(it!=history_.end()) {
                    const auto& previous=it->second;
                    const double dt=raw.seen-previous.observed;
                    if(dt>=.01 && dt<=horizon_s) {
                        next.velocity={(point.world.east-previous.position.east)/dt,
                            (point.world.north-previous.position.north)/dt,
                            (point.world.up-previous.position.up)/dt};
                        next.has_velocity=gui_point(next.velocity);
                    }
                }
                history_[point.id]=next;it=history_.find(point.id);
            } else if(it!=history_.end()) it->second.received=raw.seen;
            const double silence=now-raw.seen;
            if(silence<=gap_delay_s) continue;
            // Do not extrapolate sender predictions or extend their lease.
            if(raw.track_state!=1 || it==history_.end() || !it->second.has_velocity ||
                now-it->second.observed>horizon_s) {
                point.valid=false;point.status=2;continue;
            }
            const auto& h=it->second;
            point.world={h.position.east+h.velocity.east*silence,
                h.position.north+h.velocity.north*silence,
                h.position.up+h.velocity.up*silence};
            point.relative={point.world.east-origin.east,point.world.north-origin.north,point.world.up-origin.up};
            point.valid=gui_point(point.world) && gui_point(point.relative);
            if(!point.valid) point.status=2;
        }
        return result;
    }
};
} // namespace c2::rdr
