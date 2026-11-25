#pragma once
#include "manual_sequence.hpp"
#include "installation_config.hpp"
#include "fcs_geometry.hpp"
#include <algorithm>
#include <functional>
#include <utility>
#include <vector>

// Camera batch for GUI 37d7ab9. Only explicit operator requests move/capture.
// Freeze a fresh, private observation set BEFORE STOP; the display retention
// cache is never a source of control coordinates. Legacy Fire/Hit names are ABI.
class BatchSequence {
public:
    using Clock=ManualSequence::Clock;
    using Result=ManualSequence::Result;
    using P=ManualSequence::Phase;
    enum class Phase { idle, stopping, paused, resuming, observing, fault };
    struct Row {
        ManualSequence sequence; bool frozen{};
        double photo_distance_m{},auto_pan{},auto_tilt{};
        double photo_pan_rate{},photo_tilt_rate{},photo_seen{};
    };
    struct PhotoTarget {
        uint32_t id{}; double pan{}, tilt{}, distance_m{};
        double pan_rate{},tilt_rate{},seen{};
    };
    struct Context {
        const c2::rdr::State& rdr;
        const c2::fcs::Snapshot& fcs;
        const std::vector<c2::rdr::CorrectedTarget>& points;
        const InstallationConfig& installation;
        Clock::time_point now;
        double sample_now;
        bool scanning{}, legacy_owned{}, interlock_idle{};
    };
    struct Actions {
        std::function<Result()> stop, resume, capture, off;
        std::function<Result(double,double)> point; // tilt, pan
        std::function<Clock::time_point()> clock=Clock::now;
    };
    bool active() const { return !rows_.empty(); }
    Phase phase() const { return phase_; }
    const auto& rows() const { return rows_; }
    bool photo_pause_valid(const Context& context) const {
        return phase_==Phase::paused&&ManualSequence::stopped(context.rdr)&&held_pose_matches(context.rdr)&&
            common_error(context)==Result::success;
    }
    std::vector<PhotoTarget> photo_targets() const {
        std::vector<PhotoTarget> targets;
        if(phase_!=Phase::paused||current_||order_.size()!=rows_.size())return targets;
        targets.reserve(order_.size());
        for(const auto id:order_) {
            const auto it=rows_.find(id);
            if(it==rows_.end()||!it->second.frozen||it->second.sequence.phase!=P::registered||
               !std::isfinite(it->second.photo_distance_m)||it->second.photo_distance_m<=.001)
                return {};
            const auto& s=it->second.sequence;
            targets.push_back({id,s.pan,s.tilt,it->second.photo_distance_m,
                it->second.photo_pan_rate,it->second.photo_tilt_rate,it->second.photo_seen});
        }
        return targets;
    }
    unsigned state(uint32_t id) const {
        const auto it=rows_.find(id);return it==rows_.end()?0:it->second.sequence.state();
    }
    Result reason(uint32_t id) const {
        const auto it=rows_.find(id);return it==rows_.end()?Result::not_assigned:it->second.sequence.reason;
    }
    bool take_off_request() { return std::exchange(off_requested_,false); }
    void fail(Result reason) {
        if(!active())return;
        for(auto& [id,row]:rows_)if(!row.sequence.terminal()||row.sequence.awaiting_observation())row.sequence.fail(reason);
        if(phase_!=Phase::fault)off_requested_=true;
        phase_=Phase::fault;
    }
    unsigned flags(const Context& c) const {
        if(!active()&&c.legacy_owned)return 0;
        if(phase_==Phase::fault||common_error(c)!=Result::success)return 0;
        const bool idle_fcs=(c.fcs.status->flags&14)==0;
        const bool paused=phase_==Phase::paused&&ManualSequence::stopped(c.rdr);
        const bool settled=!current_||rows_.at(current_).sequence.phase==P::paused;
        return (paused?1u:0u)|(paused&&idle_fcs&&settled?2u:0u)|
            (!current_&&idle_fcs&&((phase_==Phase::idle&&c.interlock_idle)||paused)?4u:0u)|(idle_fcs?8u:0u);
    }
    void tick(const Context& c) {
        if(!active()||phase_==Phase::fault)return;
        const auto error=common_error(c);
        if(error!=Result::success){fail(error);return;}
        for(const auto& [id,row]:rows_) {
            if((!row.sequence.terminal()||row.sequence.awaiting_observation())&&row.sequence.generation!=c.rdr.generation){
                fail(Result::lidar_disconnected);return;
            }
        }
        if(phase_==Phase::stopping) {
            if(c.rdr.lidar_result>1||c.now-began_>=std::chrono::duration<double>(c2::rdr::lidar_command_timeout_seconds)){
                fail(stop_error(c));return;
            }
            if(ManualSequence::stopped(c.rdr)) {
                // ED07 itself may move the RDR; establish its held pose only
                // after the matching STOP completes, not at request time.
                if(c.rdr.pan==-32768||c.rdr.tilt==-32768){fail(Result::target_changed);return;}
                held_pan_=c.rdr.pan;held_tilt_=c.rdr.tilt;
                phase_=Phase::paused;
            }
        } else if(phase_==Phase::paused) {
            if(!ManualSequence::stopped(c.rdr)){fail(stop_error(c));return;}
            if(!held_pose_matches(c.rdr)){fail(Result::target_changed);return;}
            if(current_) {
                auto& s=rows_.at(current_).sequence;
                if(s.phase==P::aiming&&(c.fcs.status->flags&12)){fail(Result::fcs_off_unconfirmed);return;}
                s.tick(c.rdr,c.fcs,true,c.now,c.sample_now);
                if(s.phase==P::aimed){s.paused_at=c.now;s.enter(P::paused,c.now);}
                if(s.phase==P::fired)current_=0; // completed row survives while next object uses FCS
                else if(s.phase==P::failed){fail(s.reason);return;}
            } else if(c.fcs.status->flags&14){fail(Result::fcs_off_unconfirmed);return;}
        } else if(phase_==Phase::resuming) {
            if(c.fcs.status->flags&14){fail(Result::fcs_off_unconfirmed);return;}
            if(c.rdr.lidar_result>1||c.now-began_>=std::chrono::duration<double>(c2::rdr::lidar_command_timeout_seconds)){
                fail(Result::lidar_resume_unconfirmed);return;
            }
            if(ManualSequence::resumed(c.rdr)) {
                for(auto& [id,row]:rows_)if(row.sequence.phase==P::resuming){
                    row.sequence.resume_sample=c.sample_now;row.sequence.result_started=true;
                    row.sequence.enter(P::observing,c.now);
                }
                phase_=Phase::observing;
            }
        }
        if(phase_==Phase::observing||phase_==Phase::idle) {
            const bool has_pending=std::any_of(rows_.begin(),rows_.end(),[](const auto& pair){return pair.second.sequence.awaiting_observation();});
            if(!has_pending){phase_=Phase::idle;return;}
            if(!ManualSequence::resumed(c.rdr)){fail(Result::lidar_unknown);return;}
            bool pending=false;
            for(auto& [id,row]:rows_)if(row.sequence.awaiting_observation()){
                row.sequence.tick(c.rdr,c.fcs,false,c.now,c.sample_now);
                pending|=row.sequence.phase==P::observing;
            }
            if(!pending)phase_=Phase::idle;
        }
    }
    Result command(uint32_t command,uint32_t id,const Context& c,const Actions& actions,
                   std::optional<FcsPointAngles> aim_override={}, uint32_t* missing_observation=nullptr) {
        if(!c2::ipc::is_batch_command(command)||id>65535||(!id&&command!=1200))return Result::invalid_request;
        // Discovery/query success is independent of device readiness. GUI uses
        // connected and state/flags separately, and ignores non-success status replies.
        // Let a new GUI finish an already-owned legacy manual run via its fallback.
        if(command==1200)return !id&&c.legacy_owned?Result::not_supported:Result::success;
        if(c.legacy_owned)return Result::device_busy;
        auto it=rows_.find(id);
        if(command==1202) {
            if(it==rows_.end())return Result::success; // repeated cancellation
            const auto p=it->second.sequence.phase;
            if(p==P::aiming||p==P::firing||p==P::stopping_capture||p==P::resuming||p==P::observing||phase_==Phase::stopping||phase_==Phase::resuming)
                return Result::status_pending;
            // Removing a waiting/completed row must not interrupt another capture.
            if(current_==id||rows_.size()==1||phase_==Phase::fault) {
                const auto result=actions.off();if(result!=Result::success)return result;
            }
            if(current_==id)current_=0;
            rows_.erase(it);
            order_.erase(std::remove(order_.begin(),order_.end(),id),order_.end());
            if(rows_.empty()){phase_=Phase::idle;off_requested_=false;order_.clear();}
            return Result::success;
        }
        if(phase_==Phase::fault)return Result::lidar_recovery_required;
        const auto error=common_error(c);if(error!=Result::success)return error;
        if(command==1201) {
            if(it!=rows_.end())return it->second.sequence.phase==P::registered?Result::success:Result::already_assigned;
            if(rows_.size()>=c2::ipc::target_capacity)return Result::already_assigned;
            if(phase_!=Phase::idle||!c.interlock_idle||(c.fcs.status->flags&14))return Result::device_busy;
            if(!live_point(c,id))return Result::target_pending;
            Row row;row.sequence.target=id;row.sequence.generation=c.rdr.generation;
            row.sequence.enter(P::registered,c.now);rows_.emplace(id,row);order_.push_back(id);return Result::success;
        }
        if(it==rows_.end())return Result::not_assigned;
        auto& row=it->second;auto& s=row.sequence;
        if(command==1204) {
            if(phase_==Phase::stopping||phase_==Phase::paused)return Result::success;
            if(phase_!=Phase::idle||s.phase!=P::registered)return Result::invalid_request;
            if(!c.interlock_idle||(c.fcs.status->flags&14))return Result::device_busy;
            // Build the whole snapshot transactionally; failed validation must
            // neither send STOP nor leave a partially frozen set.
            auto frozen=rows_;
            for(auto& [key,item]:frozen)if(item.sequence.phase==P::registered) {
                const auto* point=live_point(c,key);
                if(!point) {
                    if(missing_observation)*missing_observation=key;
                    return Result::target_pending;
                }
                const auto vector=fcs_target_vector(point->world.east,point->world.north,point->world.up,
                    c.installation.rdr_position,c.installation.eff_position,c.installation.fcs_geometry);
                const auto angles=fcs_point_angles(vector[0],vector[1],vector[2],
                    c.installation.fcs_angle_offset_deg[0],c.installation.fcs_angle_offset_deg[1]);
                item.auto_pan=item.sequence.pan=angles.azimuth;
                item.auto_tilt=item.sequence.tilt=angles.elevation;
                item.photo_distance_m=std::hypot(vector[0],vector[1],vector[2]);
                item.photo_pan_rate=item.photo_tilt_rate=0;
                item.photo_seen=c.rdr.targets.at(uint16_t(key)).seen;
                if(point->velocity_valid) {
                    const auto future=fcs_target_angles(
                        point->world.east+point->velocity.east,
                        point->world.north+point->velocity.north,
                        point->world.up+point->velocity.up,
                        c.installation.rdr_position,c.installation.eff_position,
                        c.installation.fcs_angle_offset_deg[0],c.installation.fcs_angle_offset_deg[1],
                        c.installation.fcs_geometry);
                    item.photo_pan_rate=std::remainder(future.azimuth-angles.azimuth,360.);
                    item.photo_tilt_rate=future.elevation-angles.elevation;
                }
                item.sequence.capture_reference=c2::rdr::sensor_to_base(c.rdr.targets.at(uint16_t(key)));
                item.frozen=true;
            }
            rows_=std::move(frozen);phase_=Phase::stopping;began_=c.now;
            return perform(actions.stop,stop_error(c));
        }
        if(command==1203) {
            if(current_==id&&s.phase==P::aiming)return Result::success;
            const bool reaim=current_==id&&s.phase==P::paused;
            if((!reaim&&s.phase!=P::registered)||phase_!=Phase::paused||!row.frozen)return Result::invalid_request;
            if((current_&&!reaim)||(c.fcs.status->flags&14))return Result::device_busy;
            if(!ManualSequence::stopped(c.rdr))return stop_error(c);
            if(aim_override) {
                s.pan=aim_override->azimuth;
                s.tilt=aim_override->elevation;
            } else {s.pan=row.auto_pan;s.tilt=row.auto_tilt;}
            // Own the slot before issuing IO, including unknown/exception outcomes.
            current_=id;s.enter(P::aiming,c.now);
            return perform([&]{return actions.point(s.tilt,s.pan);},Result::device_error);
        }
        if(command==1205) {
            if(s.phase==P::firing||s.phase==P::stopping_capture||s.phase==P::fired)return Result::success;
            if(current_!=id||s.phase!=P::paused||phase_!=Phase::paused)return Result::invalid_request;
            if(!ManualSequence::stopped(c.rdr))return stop_error(c);
            if(c.fcs.status->flags&14)return Result::device_busy;
            if(fcs_pan_error(c.fcs.status->azimuth,s.pan)>1||std::abs(c.fcs.status->elevation-s.tilt)>1)
                return Result::alignment_required;
            s.output_seen=false;s.enter(P::firing,c.now);
            return perform(actions.capture,Result::device_error);
        }
        if(command==uint32_t(c2::ipc::Command::stop_batch_capture)) {
            if(s.phase==P::stopping_capture||s.phase==P::fired)return Result::success;
            if(current_!=id||s.phase!=P::firing||phase_!=Phase::paused)return Result::invalid_request;
            const auto result=perform(actions.off,Result::fcs_off_unconfirmed);
            if(result==Result::success)s.enter(P::stopping_capture,actions.clock());
            return result;
        }
        if(command==1206) {
            if(phase_==Phase::resuming||phase_==Phase::observing)return Result::success;
            if(phase_==Phase::idle&&(s.phase==P::alive||s.phase==P::hit))return Result::success;
            if(phase_!=Phase::paused)return Result::invalid_request;
            if(!(flags(c)&2))return Result::status_pending;
            for(auto& [key,item]:rows_) {
                if(item.sequence.phase==P::fired)item.sequence.enter(P::resuming,c.now);
                else if(item.sequence.phase==P::aiming||item.sequence.phase==P::paused)
                    item.sequence.enter(P::registered,c.now);
                item.frozen=false;
            }
            current_=0;phase_=Phase::resuming;began_=c.now;
            return perform(actions.resume,Result::lidar_resume_unconfirmed);
        }
        if(command==1207) {
            if(phase_!=Phase::idle)return Result::device_busy;
            if(s.phase==P::registered)return Result::success;
            if(s.phase!=P::alive)return Result::invalid_request;
            if(!live_point(c,id))return Result::target_pending;
            if(!c.interlock_idle||(c.fcs.status->flags&14))return Result::device_busy;
            s=ManualSequence{};s.target=id;s.generation=c.rdr.generation;s.enter(P::registered,c.now);
            row.frozen=false;return Result::success;
        }
        return Result::not_supported;
    }
private:
    std::map<uint32_t,Row> rows_;
    std::vector<uint32_t> order_;
    Phase phase_{Phase::idle};
    uint32_t current_{};
    Clock::time_point began_{};
    int held_pan_{},held_tilt_{};
    bool off_requested_{};
    bool held_pose_matches(const c2::rdr::State& r) const {
        if(r.pan==-32768||r.tilt==-32768)return false;
        // RDR reports tenths of a degree. Allow one readback count per axis:
        // the 2026-11-16 trace changed 1002 -> 1003 while stopped. Keep the
        // original STOP reference so repeated small steps cannot hide drift.
        constexpr int readback_tolerance_ddeg=1;
        const int pan_delta=std::abs(r.pan-held_pan_)%3600;
        return std::min(pan_delta,3600-pan_delta)<=readback_tolerance_ddeg&&
            std::abs(r.tilt-held_tilt_)<=readback_tolerance_ddeg;
    }
    static Result common_error(const Context& c) {
        if(!c.rdr.connected)return Result::lidar_disconnected;
        if(c.rdr.thermal_fault())return Result::lidar_thermal_fault;
        if(!c.rdr.reporting||c.rdr.mode!=2)return Result::target_pending;
        if(!c.fcs.connected||!c.fcs.status||c.now-c.fcs.status_received_at>std::chrono::seconds(2))return Result::status_pending;
        if(!(c.fcs.status->flags&1)||(c.fcs.status->flags&16)||
           c.fcs.status->azimuth==-32768||c.fcs.status->elevation==-32768)return Result::device_fault;
        return Result::success;
    }
    static Result stop_error(const Context& c) {
        return c.rdr.thermal_warning()?Result::lidar_stop_warning_unconfirmed:Result::lidar_stop_unconfirmed;
    }
    static const c2::rdr::CorrectedTarget* live_point(const Context& c,uint32_t id) {
        if(!c.scanning)return nullptr;
        for(const auto& p:c.points)if(p.id==id&&
            c2::rdr::control_observation_ready(c.rdr,p,c.sample_now))return &p;
        return nullptr;
    }
    Result perform(const std::function<Result()>& action,Result exception_reason) {
        try {const auto result=action();if(result!=Result::success)fail(result);return result;}
        catch(...){fail(exception_reason);throw;}
    }
};
