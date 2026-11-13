#pragma once
#include "rdr_coordinates.hpp"
#include <set>

namespace c2::rdr {
// Display-only memory across an explicitly requested optical pause. Identity
// association belongs to RDR; this class never aliases a new ID to an old one.
class ObservationRetention {
    std::map<uint16_t, CorrectedTarget> held_;
    std::set<uint16_t> observed_;
    uint64_t generation_{};
    BasePoint origin_{};
    double started_{}, resumed_{};
    bool active_{}, resumed_flag_{};
public:
    static constexpr double pause_limit_s=10, reacquire_limit_s=3;
    void reset() { held_.clear();observed_.clear();active_=resumed_flag_=false; }
    void begin(const State& r,const std::vector<CorrectedTarget>& points,BasePoint origin,double now) {
        reset();
        if(!r.connected||r.mode!=2||!r.reporting||!std::isfinite(now))return;
        active_=true;generation_=r.generation;origin_=origin;started_=now;
        for(const auto& p:points) if(p.valid&&held_.size()<10)held_[p.id]=p;
    }
    // Invalidates control coordinates in place; returned points also contain
    // retained positions with valid=false/status=4 paused or 5 reacquiring.
    std::vector<CorrectedTarget> update(const State& r,std::vector<CorrectedTarget>& control,
        BasePoint origin,double now,bool resume_confirmed) {
        if(!active_){
            // Display-cache lifetime must never bypass a physical inhibit.
            if(r.lidar_inhibited)for(auto& p:control){p.valid=false;p.status=2;}
            return control;
        }
        if(!r.connected||!r.reporting||r.mode!=2||r.generation!=generation_||
           !std::isfinite(now)||now<started_) {
            reset();for(auto& p:control){p.valid=false;p.status=2;}return control;
        }
        if(origin.east!=origin_.east||origin.north!=origin_.north||origin.up!=origin_.up){
            // Discard coordinates in the old installation frame, not the pause
            // gate or the evidence of observations already received after resume.
            held_.clear();origin_=origin;
        }
        if(!resumed_flag_&&resume_confirmed&&r.lidar_supported&&r.lidar_known&&
           !r.lidar_inhibited&&r.lidar_on) {
            if(now-started_>=pause_limit_s)held_.clear();
            resumed_flag_=true;resumed_=now;
        }
        if((!resumed_flag_&&now-started_>=pause_limit_s)||
           (resumed_flag_&&now-resumed_>=reacquire_limit_s))held_.clear();
        for(auto it=held_.begin();it!=held_.end();)
            if(r.excluded.contains(it->first))it=held_.erase(it);else ++it;
        for(auto& p:control) {
            const auto raw=r.targets.find(p.id);
            if(resumed_flag_&&!r.lidar_inhibited&&r.lidar_known&&r.lidar_on&&p.valid&&
               raw!=r.targets.end()&&raw->second.track_state==1&&raw->second.detected==1&&
               raw->second.seen>resumed_) {observed_.insert(p.id);held_.erase(p.id);}
            if(!resumed_flag_||r.lidar_inhibited||!r.lidar_known||!r.lidar_on||!observed_.contains(p.id)) {
                p.valid=false;p.status=2;
            }
        }
        std::vector<CorrectedTarget> display;
        // Keep original IDs visible first; new IDs are never substituted for them.
        for(const auto& [id,last]:held_) {
            auto p=last;p.valid=false;p.status=resumed_flag_?5:4;display.push_back(p);
        }
        // Preserve ordinary LOST records in IPC; only statuses 4/5 retain a
        // visible position when invalid. Consumers still receive loss updates.
        for(const auto& p:control) if(!held_.contains(p.id)&&display.size()<10)display.push_back(p);
        return display;
    }
};
}
