#pragma once
#include "rdr_coordinates.hpp"
#include <set>
#include <algorithm>
#include <deque>

namespace c2::rdr {
// ECS-lifetime result history for display only. Never passed to control logic.
class HitRetention {
    std::map<uint16_t, CorrectedTarget> last_;
    std::deque<CorrectedTarget> hits_; // oldest confirmation first
public:
    void observe(const std::vector<CorrectedTarget>& points,const std::set<uint16_t>& assigned) {
        std::set<uint16_t> keep=assigned;
        for(const auto& point:points) if(point.valid) {last_[point.id]=point;keep.insert(point.id);}
        for(auto it=last_.begin();it!=last_.end();)
            if(!keep.contains(it->first))it=last_.erase(it);else ++it;
    }
    void confirm(uint16_t id) {
        if(std::any_of(hits_.begin(),hits_.end(),[&](const auto& p){return p.id==id;}))return;
        const auto it=last_.find(id);if(it==last_.end())return;
        auto point=it->second;point.status=1;point.valid=false;hits_.push_back(point);
    }
    std::vector<CorrectedTarget> merge(const std::vector<CorrectedTarget>& current) {
        constexpr size_t capacity=10;
        // Current observations supersede a display-only historical result.
        // Remove the stale row permanently so subsequent loss cannot revive it.
        std::erase_if(hits_,[&](const auto& hit){
            return std::any_of(current.begin(),current.end(),[&](const auto& p){return p.id==hit.id&&p.valid;});
        });
        const auto is_hit=[&](uint16_t id) {
            return std::any_of(hits_.begin(),hits_.end(),[&](const auto& p){return p.id==id;});
        };
        std::vector<CorrectedTarget> result;
        for(const auto& p:current)
            if(!is_hit(p.id)&&(p.valid||p.status==4||p.status==5)&&result.size()<capacity)result.push_back(p);
        while(result.size()+hits_.size()>capacity)hits_.pop_front();
        result.insert(result.end(),hits_.begin(),hits_.end());
        // Ordinary loss records use only spare slots, never displace visible history.
        for(const auto& p:current) if(result.size()<capacity&&!p.valid&&p.status!=4&&p.status!=5&&
            std::none_of(result.begin(),result.end(),[&](const auto& row){return row.id==p.id;}))result.push_back(p);
        return result;
    }
};
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
    static constexpr double reacquire_limit_s=3;
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
        BasePoint origin,double now,bool resume_confirmed,bool /*completed_manual_wait*/=false) {
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
        if(!resumed_flag_&&resume_confirmed&&r.lidar_supported&&
           !r.lidar_inhibited) {
            resumed_flag_=true;resumed_=now;
        }
        // A stopped sensor cannot refresh its observations. Keep the display
        // snapshot for the entire pause, independently of command deadlines.
        if(resumed_flag_&&now-resumed_>=reacquire_limit_s)held_.clear();
        for(auto it=held_.begin();it!=held_.end();)
            if(r.excluded.contains(it->first))it=held_.erase(it);else ++it;
        for(auto& p:control) {
            const auto raw=r.targets.find(p.id);
            if(resumed_flag_&&!r.lidar_inhibited&&p.valid&&
               raw!=r.targets.end()&&raw->second.track_state==1&&raw->second.detected==1&&
               raw->second.seen>resumed_) {observed_.insert(p.id);held_.erase(p.id);}
            if(!resumed_flag_||r.lidar_inhibited||!observed_.contains(p.id)) {
                p.valid=false;p.status=2;
            }
        }
        // A fresh post-resume observation replaces the display-only pause snapshot.
        // Never merge or rename sensor IDs; control still uses fresh IDs only.
        if(resumed_flag_&&!observed_.empty())held_.clear();
        std::vector<CorrectedTarget> display;
        // Retain paused positions only until fresh post-resume observations arrive.
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
