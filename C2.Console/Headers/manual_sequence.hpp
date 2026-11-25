#pragma once
#include "protocol.hpp"
#include "c2/rdr_session.hpp"
#include "c2/fcs_client.hpp"
#include "fcs_angles.hpp"
#include "rdr_coordinates.hpp"
#include <chrono>
#include <cmath>

// One fixed educational target, owned by ECS. UI states are the published GUI
// contract; private phases distinguish an accepted request from completion.
class ManualSequence {
public:
    using Clock=std::chrono::steady_clock;
    using Result=c2::ipc::Result;
    static constexpr auto capture_completion_timeout=std::chrono::seconds(30);
    enum class Phase { none, registered, aiming, aimed, pausing, paused, firing, stopping_capture,
                       fired, resuming, observing, hit, alive, failed };
    Phase phase{Phase::none};
    uint32_t target{}; uint64_t generation{};
    double pan{},tilt{},resume_sample{};
    std::optional<c2::rdr::BasePoint> capture_reference;
    Result reason{Result::success};
    bool output_seen{}, recovery_requested{}, result_started{};
    Clock::time_point began{},paused_at{};
    unsigned failed_step{};
    uint16_t observation_candidate{};
    double candidate_seen{};
    bool late_window_closed{};
    // Preserve the original resume epoch even after publishing an uncertain result.
    static constexpr auto late_observation_limit=std::chrono::seconds(13);
    bool awaiting_observation() const {
        return phase==Phase::observing||
            (phase==Phase::failed&&reason==Result::result_unconfirmed&&result_started&&!late_window_closed);
    }
    bool active() const {return phase!=Phase::none;}
    bool terminal() const {return phase==Phase::hit||phase==Phase::alive||phase==Phase::failed;}
    unsigned state() const {
        switch(phase) {
        case Phase::registered:return 102;
        case Phase::aiming:return 110;
        case Phase::aimed:case Phase::pausing:return 103;
        case Phase::paused:return 104;
        case Phase::firing:return 105;
        case Phase::stopping_capture:return 112;
        case Phase::fired:case Phase::resuming:return 106;
        case Phase::observing:return 107;
        case Phase::hit:return 108;
        case Phase::alive:return 111;
        case Phase::failed:return 3;
        default:return 0;
        }
    }
    void enter(Phase next,Clock::time_point now) {
        phase=next;began=now;reason=Result::success;
        if(next==Phase::observing){observation_candidate=0;candidate_seen=0;late_window_closed=false;}
    }
    unsigned completed_step() const {
        if(phase==Phase::failed)return failed_step;
        if(phase==Phase::aiming)return 2;
        if(phase==Phase::stopping_capture)return 5;
        if(phase==Phase::alive)return 8;
        const auto value=state();
        return value>=102&&value<=108?value-100:0;
    }
    void fail(Result why) {if(phase!=Phase::failed)failed_step=completed_step();phase=Phase::failed;reason=why;}
    static bool stopped(const c2::rdr::State& r) {
        return r.stop_completed();
    }
    static bool resumed(const c2::rdr::State& r) {
        return !r.thermal_fault()&&r.connected&&r.mode==2&&r.reporting&&r.lidar_supported&&
            !r.lidar_inhibited&&r.lidar_request==1&&r.lidar_result==0;
    }
    void tick(const c2::rdr::State& r,const c2::fcs::Snapshot& f,bool same_target,
              Clock::time_point now,double sample_now) {
        if(!active()||(terminal()&&!awaiting_observation()))return;
        if(awaiting_observation()&&now-began>late_observation_limit){
            if(phase==Phase::observing)fail(Result::result_unconfirmed);
            late_window_closed=true;
            return;
        }
        if(!r.connected||r.generation!=generation){fail(Result::lidar_disconnected);return;}
        if(r.thermal_fault()){fail(Result::lidar_thermal_fault);return;}
        if(r.mode!=2||!r.reporting){fail(Result::target_pending);return;}
        if(!f.connected||!f.status||now-f.status_received_at>std::chrono::seconds(2)){
            fail(Result::status_pending);return;
        }
        const auto flags=f.status->flags;
        if(!(flags&1)||(flags&16)){fail(Result::device_fault);return;}
        const bool fresh=f.status_received_at>began;
        const bool aligned=fcs_pan_error(f.status->azimuth,pan)<=1&&std::abs(f.status->elevation-tilt)<=1;
        if(phase==Phase::aiming) {
            if(!same_target){fail(Result::target_changed);return;}
            if(now-began>std::chrono::seconds(60)){fail(Result::alignment_required);return;}
            if(fresh&&aligned&&!(flags&2))enter(Phase::aimed,now);
        } else if(phase==Phase::aimed) {
            if(!same_target||!aligned||(flags&14))fail(Result::alignment_required);
        } else if(phase==Phase::pausing) {
            if(r.lidar_result>1||now-began>=std::chrono::duration<double>(c2::rdr::lidar_command_timeout_seconds)){
                fail(r.thermal_warning()?Result::lidar_stop_warning_unconfirmed:Result::lidar_stop_unconfirmed);return;
            }
            if(stopped(r)&&fresh&&aligned&&!(flags&14)){
                paused_at=now;enter(Phase::paused,now);
            }
        } else if(phase==Phase::paused) {
            // Wait for an explicit operator request without a time limit.
            if(!stopped(r)||!aligned||(flags&14))fail(Result::lidar_stop_unconfirmed);
        } else if(phase==Phase::firing||phase==Phase::stopping_capture) {
            if(!stopped(r)){fail(r.thermal_warning()?Result::lidar_stop_warning_unconfirmed:Result::lidar_stop_unconfirmed);return;}
            if(fresh&&(flags&12))output_seen=true;
            // After OFF acceptance, require a newer report with both motion and output clear.
            if(fresh&&(output_seen||phase==Phase::stopping_capture)&&!(flags&14))enter(Phase::fired,now);
            else if(now-began>=capture_completion_timeout)fail(Result::fcs_off_unconfirmed);
        } else if(phase==Phase::fired) {
            // Operator-paced wait after confirmed completion; no pause deadline.
            if(!stopped(r)){fail(r.thermal_warning()?Result::lidar_stop_warning_unconfirmed:Result::lidar_stop_unconfirmed);return;}
            if(flags&14){fail(Result::fcs_off_unconfirmed);return;}
        } else if(phase==Phase::resuming) {
            if(flags&14){fail(Result::fcs_off_unconfirmed);return;}
            if(r.lidar_result>1||now-began>=std::chrono::duration<double>(c2::rdr::lidar_command_timeout_seconds)){
                fail(Result::lidar_resume_unconfirmed);return;
            }
            if(resumed(r)){resume_sample=sample_now;result_started=true;enter(Phase::observing,now);}
        } else if(awaiting_observation()) {
            if(!resumed(r)){fail(Result::lidar_unknown);return;}
            if(flags&14){fail(Result::fcs_off_unconfirmed);return;}
            const auto observed=[&](const auto& t) {
                return t.seen>resume_sample&&t.seen<=sample_now&&sample_now-t.seen<=3&&
                    t.track_state==1&&t.detected==1&&!r.excluded.contains(t.id);
            };
            const auto it=r.targets.find(uint16_t(target));
            if(it!=r.targets.end()&&observed(it->second)){enter(Phase::alive,now);return;}
            // A unique nearby replacement requires two distinct observations.
            // Repeated ECS ticks over one packet are not additional evidence.
            const c2::rdr::Target* candidate=nullptr;
            bool ambiguous=false;
            if(capture_reference)for(const auto& [id,t]:r.targets) {
                const auto& ref=*capture_reference;
                const auto point=c2::rdr::sensor_to_base(t);
                if(point&&observed(t)&&
                   std::hypot(point->east-ref.east,point->north-ref.north,point->up-ref.up)<=0.15) {
                    if(candidate){ambiguous=true;break;}
                    candidate=&t;
                }
            }
            if(candidate&&!ambiguous) {
                if(observation_candidate==candidate->id&&candidate->seen>candidate_seen&&
                   candidate->seen-candidate_seen<=3){enter(Phase::alive,now);return;}
                observation_candidate=candidate->id;candidate_seen=candidate->seen;
            } else {observation_candidate=0;candidate_seen=0;}
            if(phase==Phase::observing&&now-began>=std::chrono::seconds(3)) {
                // A target that is still absent after the post-resume observation
                // window is considered a confirmed HIT.
                enter(Phase::hit,now);
            }
        }
    }
};
