#pragma once
#include "batch_sequence.hpp"
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <numbers>
#include <vector>

// Operator-started camera coverage run over an already frozen batch. EF04(1)
// acknowledgement is represented by Actions::capture returning success. Unknown
// outcomes are terminal and are never retried.
class ContinuousPhotoSequence {
public:
    using Clock=std::chrono::steady_clock;
    using Result=c2::ipc::Result;
    enum class Phase : std::uint32_t { idle, running, completed, stopped, failed };
    struct Entry { std::uint32_t target{}; double pan{},tilt{}; };
    struct Context {
        const c2::fcs::Snapshot& fcs;
        bool batch_paused{};
        Clock::time_point now;
    };
    struct Actions {
        std::function<Result(double,double)> point; // tilt, pan
        std::function<Result()> capture, off;
        std::function<Clock::time_point()> clock=Clock::now;
    };

    Phase phase() const {return phase_;}
    Result reason() const {return reason_;}
    std::uint32_t total() const {return static_cast<std::uint32_t>(plan_.size());}
    std::uint32_t confirmed() const {return confirmed_;}
    std::uint32_t current_target() const {
        return index_<plan_.size()?plan_[index_].target:0;
    }
    bool running() const {return phase_==Phase::running;}
    bool holds_batch() const {return running()||holds_batch_;}
    const std::vector<Entry>& plan() const {return plan_;}

    Result start(const std::vector<BatchSequence::PhotoTarget>& targets,double lead_seconds,unsigned repeats,
                 double sweep_extent_m,double sample_now,
                 const Context& context,const Actions& actions) {
        if(holds_batch())return Result::device_busy;
        reset();
        if(!context.batch_paused||targets.empty()||!ready(context)||(context.fcs.status->flags&14)||
           !std::isfinite(lead_seconds)||lead_seconds<0||lead_seconds>2||
           !std::isfinite(sweep_extent_m)||sweep_extent_m<=0||sweep_extent_m>1||
           !std::isfinite(sample_now)||repeats<1||repeats>20)
            return Result::invalid_request;
        for(const auto& target:targets)
            if(!target.id||!std::isfinite(target.pan)||!std::isfinite(target.tilt)||
               !std::isfinite(target.distance_m)||target.distance_m<=.001||
               !std::isfinite(target.pan_rate)||!std::isfinite(target.tilt_rate)||
               !std::isfinite(target.seen)||target.seen>sample_now)return Result::invalid_request;
        plan_=make_plan(targets,lead_seconds,repeats,sweep_extent_m,sample_now);
        if(plan_.empty())return Result::invalid_request;
        phase_=Phase::running;
        return begin_move(context,actions);
    }

    void tick(const Context& context,const Actions& actions) {
        if(!running()) {
            if(holds_batch_&&context.fcs.connected&&context.fcs.status&&
               context.fcs.status_received_at>off_status_floor_&&!(context.fcs.status->flags&14))
                holds_batch_=false;
            return;
        }
        if(!context.batch_paused){terminate(Phase::failed,Result::lidar_stop_unconfirmed,context,actions);return;}
        if(!ready(context)){terminate(Phase::failed,context.fcs.connected?Result::status_pending:Result::asset_unavailable,context,actions);return;}
        if(context.now-began_>std::chrono::seconds(60)) {
            terminate(Phase::failed,Result::alignment_required,context,actions);return;
        }
        const auto& status=*context.fcs.status;
        if(status.flags&16) {terminate(Phase::failed,Result::device_fault,context,actions);return;}
        if(context.fcs.status_received_at<=status_floor_||(status.flags&2))return;
        const auto& entry=plan_[index_];
        // The FCS position feedback can settle up to 0.4 degrees from the
        // requested tenth-degree command. Accept that hardware deadband while
        // retaining a small floating-point comparison margin.
        if(fcs_pan_error(status.azimuth,entry.pan)>.400001||std::abs(status.elevation-entry.tilt)>.400001)return;
        Result result;
        try {result=actions.capture();}
        catch(...) {terminate(Phase::failed,Result::device_error,context,actions);throw;}
        if(result!=Result::success){terminate(Phase::failed,result,context,actions);return;}
        ++confirmed_;
        if(confirmed_==plan_.size()) {terminate(Phase::completed,Result::success,context,actions);return;}
        ++index_;
        (void)begin_move({context.fcs,context.batch_paused,actions.clock()},actions);
    }

    Result stop(const Context& context,const Actions& actions) {
        if(!running())return phase_==Phase::failed?reason_:Result::success;
        terminate(Phase::stopped,Result::success,context,actions);
        return reason_;
    }

    static std::vector<Entry> make_plan(const std::vector<BatchSequence::PhotoTarget>& targets,
                                        double lead_seconds,unsigned repeats,double sweep_extent_m,
                                        double sample_now) {
        if(targets.empty()||!std::isfinite(lead_seconds)||lead_seconds<0||lead_seconds>2||
           !std::isfinite(sweep_extent_m)||sweep_extent_m<=0||sweep_extent_m>1||
           !std::isfinite(sample_now)||repeats<1||repeats>20)return {};
        std::vector<Entry> result;
        result.reserve(targets.size()*repeats);
        for(const auto& target:targets) {
            if(!target.id||!std::isfinite(target.pan)||!std::isfinite(target.tilt)||
               !std::isfinite(target.pan_rate)||!std::isfinite(target.tilt_rate)||
               !std::isfinite(target.seen)||target.seen>sample_now)return {};
            const double horizon=std::clamp(sample_now-target.seen+lead_seconds,0.,3.);
            const double center_pan=target.pan_rate*horizon;
            const double center_tilt=target.tilt_rate*horizon;
            const double speed=std::hypot(target.pan_rate,target.tilt_rate);
            constexpr double inverse_sqrt_two=.7071067811865475244;
            // Rotate the observed image-plane motion by +45 degrees. With no
            // trustworthy motion, fall back to the screen's rising diagonal.
            const double sweep_pan=speed>1e-6?
                (target.pan_rate-target.tilt_rate)/speed*inverse_sqrt_two:inverse_sqrt_two;
            const double sweep_tilt=speed>1e-6?
                (target.pan_rate+target.tilt_rate)/speed*inverse_sqrt_two:inverse_sqrt_two;
            const double angular_extent=std::atan2(sweep_extent_m,target.distance_m)*180/std::numbers::pi;
            for(unsigned repeat=0;repeat<repeats;++repeat) {
                const double position=repeats==1?0.:2.*repeat/(repeats-1)-1.;
                result.push_back(entry(target,center_pan+sweep_pan*angular_extent*position,
                    center_tilt+sweep_tilt*angular_extent*position));
            }
        }
        return result;
    }

private:
    Phase phase_{Phase::idle};
    Result reason_{Result::success};
    std::vector<Entry> plan_;
    std::size_t index_{};
    std::uint32_t confirmed_{};
    Clock::time_point began_{},status_floor_{};
    Clock::time_point off_status_floor_{};
    bool holds_batch_{};

    static Entry entry(const BatchSequence::PhotoTarget& target,double pan_offset,double tilt_offset) {
        if(!std::isfinite(target.pan)||!std::isfinite(target.tilt)||
           !std::isfinite(target.distance_m)||target.distance_m<=.001)return {};
        double pan=std::round((target.pan+pan_offset)*10)/10;
        pan=std::fmod(pan,360.);if(pan<0)pan+=360.;
        const double tilt=std::round((target.tilt+tilt_offset)*10)/10;
        return {target.id,pan,tilt};
    }
    static bool ready(const Context& context) {
        if(!context.fcs.connected||!context.fcs.status||
           context.now-context.fcs.status_received_at>std::chrono::seconds(2))return false;
        const auto flags=context.fcs.status->flags;
        return (flags&1)&&!(flags&16)&&context.fcs.status->azimuth!=-32768&&
            context.fcs.status->elevation!=-32768;
    }
    Result begin_move(const Context& context,const Actions& actions) {
        const auto& entry=plan_[index_];
        Result result;
        try {result=actions.point(entry.tilt,entry.pan);}
        catch(...) {terminate(Phase::failed,Result::device_error,context,actions);throw;}
        if(result!=Result::success)terminate(Phase::failed,result,context,actions);
        else {
            // The floor must be sampled after the accepted EF03 reply. A FE02
            // received while EF03 was in flight describes an unknown command
            // boundary and must not authorize EF04(1).
            status_floor_=actions.clock();
            began_=status_floor_;
        }
        return result;
    }
    void terminate(Phase requested,Result reason,const Context& context,const Actions& actions) {
        Result off_result=Result::success;
        try {off_result=actions.off();}
        catch(...) {off_result=Result::fcs_off_unconfirmed;}
        phase_=off_result==Result::success?requested:Phase::failed;
        reason_=off_result==Result::success?reason:Result::fcs_off_unconfirmed;
        // Keep owning the paused batch until a status received strictly after
        // the EF04(0) reply proves output and motion are both clear.
        off_status_floor_=actions.clock();holds_batch_=true;
    }
    void reset() {
        phase_=Phase::idle;reason_=Result::success;plan_.clear();index_=0;confirmed_=0;
        began_={};status_floor_={};off_status_floor_={};holds_batch_=false;
    }
};
