#pragma once
#include "protocol.hpp"
#include "c2/rdr_session.hpp"
#include "c2/fcs_client.hpp"
#include <chrono>
#include <cmath>

// One fixed educational target, owned by ECS. UI states are the published GUI
// contract; private phases distinguish an accepted request from completion.
class ManualSequence {
public:
    using Clock=std::chrono::steady_clock;
    using Result=c2::ipc::Result;
    enum class Phase { none, registered, aiming, aimed, pausing, paused, firing,
                       fired, resuming, observing, hit, alive, failed };
    Phase phase{Phase::none};
    uint32_t target{}; uint64_t generation{};
    double pan{},tilt{},resume_sample{};
    Result reason{Result::success};
    bool output_seen{}, recovery_requested{}, result_started{};
    Clock::time_point began{},paused_at{};
    bool active() const {return phase!=Phase::none;}
    bool terminal() const {return phase==Phase::hit||phase==Phase::alive||phase==Phase::failed;}
    unsigned state() const {
        switch(phase) {
        case Phase::registered:return 102;
        case Phase::aiming:return 110;
        case Phase::aimed:case Phase::pausing:return 103;
        case Phase::paused:return 104;
        case Phase::firing:return 105;
        case Phase::fired:case Phase::resuming:return 106;
        case Phase::observing:return 107;
        case Phase::hit:return 108;
        case Phase::alive:return 111;
        case Phase::failed:return 3;
        default:return 0;
        }
    }
    void enter(Phase next,Clock::time_point now) {phase=next;began=now;reason=Result::success;}
    void fail(Result why) {phase=Phase::failed;reason=why;}
    static bool stopped(const c2::rdr::State& r) {
        return r.connected&&r.lidar_supported&&r.lidar_known&&!r.lidar_on&&
            r.lidar_inhibited&&r.lidar_request==0&&r.lidar_result==0;
    }
    static bool resumed(const c2::rdr::State& r) {
        return r.connected&&r.mode==2&&r.reporting&&r.lidar_supported&&r.lidar_known&&
            r.lidar_on&&!r.lidar_inhibited&&r.lidar_request==1&&r.lidar_result==0;
    }
    void tick(const c2::rdr::State& r,const c2::fcs::Snapshot& f,bool same_target,
              Clock::time_point now,double sample_now) {
        if(!active()||terminal())return;
        if(!r.connected||r.generation!=generation){fail(Result::lidar_disconnected);return;}
        if(r.mode!=2||!r.reporting){fail(Result::target_pending);return;}
        if(!f.connected||!f.status||now-f.status_received_at>std::chrono::seconds(2)){
            fail(Result::status_pending);return;
        }
        const auto flags=f.status->flags;
        if(!(flags&1)||(flags&16)){fail(Result::device_fault);return;}
        const bool fresh=f.status_received_at>began;
        const bool aligned=std::abs(f.status->azimuth-pan)<=1&&std::abs(f.status->elevation-tilt)<=1;
        if(phase==Phase::aiming) {
            if(!same_target){fail(Result::target_changed);return;}
            if(now-began>std::chrono::seconds(60)){fail(Result::alignment_required);return;}
            if(fresh&&aligned&&!(flags&2))enter(Phase::aimed,now);
        } else if(phase==Phase::aimed) {
            if(!same_target||!aligned||(flags&14))fail(Result::alignment_required);
        } else if(phase==Phase::pausing) {
            if(r.lidar_result>1||now-began>=std::chrono::duration<double>(c2::rdr::lidar_command_timeout_seconds)){
                fail(Result::lidar_stop_unconfirmed);return;
            }
            if(stopped(r)&&fresh&&aligned&&!(flags&14))enter(Phase::paused,now);
        } else if(phase==Phase::paused) {
            if(!stopped(r)||!aligned||(flags&14))fail(Result::lidar_stop_unconfirmed);
        } else if(phase==Phase::firing) {
            if(!stopped(r)){fail(Result::lidar_stop_unconfirmed);return;}
            if(fresh&&(flags&12))output_seen=true;
            if(fresh&&output_seen&&!(flags&14))enter(Phase::fired,now);
            else if(now-began>std::chrono::seconds(8))fail(Result::fcs_off_unconfirmed);
        } else if(phase==Phase::resuming) {
            if(flags&14){fail(Result::fcs_off_unconfirmed);return;}
            if(r.lidar_result>1||now-began>=std::chrono::duration<double>(c2::rdr::lidar_command_timeout_seconds)){
                fail(Result::lidar_resume_unconfirmed);return;
            }
            if(resumed(r)){resume_sample=sample_now;result_started=true;enter(Phase::observing,now);}
        } else if(phase==Phase::observing) {
            if(!resumed(r)){fail(Result::lidar_unknown);return;}
            const auto it=r.targets.find(uint16_t(target));
            if(it!=r.targets.end()&&it->second.seen>resume_sample&&
               it->second.track_state==1&&it->second.detected==1){enter(Phase::alive,now);return;}
            // A packet gap, LOST, or a changed ID is NOT evidence of a hit.
            // No hit is synthesized without an independently confirmed result.
            if(now-began>=std::chrono::seconds(3))fail(Result::device_error);
        }
        if((phase==Phase::pausing||phase==Phase::paused||phase==Phase::firing||phase==Phase::fired)&&
           now-paused_at>=std::chrono::seconds(10))fail(Result::lidar_recovery_required);
    }
};
