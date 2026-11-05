#pragma once
#include "c2/rdr_session.hpp"
#include "rdr_coordinates.hpp"
#include "protocol.hpp"
#include <cmath>
#include <functional>
#include <numbers>
#include <string>

// Adapter used by the existing live station's tick/command/publish loop.
// Driver is the 24-byte UDP Client in production, an in-process board in tests.
template<class Driver> class RdrControl {
    Driver& driver_;
    std::function<void(const std::string&)> log_;
    uint32_t command_{}, request_{};
    uint16_t previous_result_{};
    double gui_seen_{};
    bool stopped_{}, heartbeat_seen_{};
    enum class StartPhase { none, reporting, starting };
    StartPhase start_phase_{StartPhase::none};
    uint64_t start_generation_{};
    bool stop_requested_{};
    static uint32_t result(uint16_t code) {
        switch(code){case 0:return 0;case 1:return 7;case 2:case 6:return 1;
        case 3:return 2;case 4:return 12;case 9:return 13;default:return 6;}
    }
public:
    RdrControl(Driver& driver, std::function<void(const std::string&)> log, double now)
        : driver_(driver),log_(std::move(log)),gui_seen_(now) {}
    void tick(double now) {
        driver_.tick();auto state=driver_.snapshot();
        if (start_phase_ != StartPhase::none) {
            if (!state.connected || state.generation != start_generation_ || state.last_result > 1)
                start_phase_ = StartPhase::none;
            else if (start_phase_ == StartPhase::reporting && state.reporting && state.last_result == 0) {
                const auto code = driver_.command(0xED06, 1);
                start_phase_ = code <= 1 ? StartPhase::starting : StartPhase::none;
                if (code > 1) log_("GUI RDR start rejected after report confirmation");
            } else if (start_phase_ == StartPhase::starting && state.mode == 2)
                start_phase_ = StartPhase::none;
        }
        if (state.mode == 0 && state.last_result != 1) stop_requested_ = false;
        if(heartbeat_seen_ && now-gui_seen_>4 && !stopped_ && state.connected) {
            driver_.command(0xED06,0);stopped_=true;stop_requested_=true;
            start_phase_=StartPhase::none;
            log_("GUI heartbeat missing: requested surveillance stop");
        }
        state=driver_.snapshot();
        if(previous_result_!=state.last_result) {
            previous_result_=state.last_result;
            log_("RDR operation="+std::to_string(command_)+" board_result="+std::to_string(previous_result_));
        }
    }
    bool operating() {
        return operating(driver_.snapshot());
    }
    bool operating(const c2::rdr::State& state) const {
        return state.connected && state.mode==2 && state.reporting && !stop_requested_;
    }
    void cancel_start() { start_phase_=StartPhase::none; }
    // GUI v3 acknowledges acceptance; actual completion is published in isOperating.
    c2::ipc::PipeResponse set_operating(const c2::ipc::PipeCommand& q) {
        c2::ipc::PipeResponse out;out.request_id=q.request_id;out.command=q.command;
        out.fcs_id=q.fcs_id;out.target_id=q.target_id;
        const auto state=driver_.snapshot();out.connected=state.connected;
        if (!q.request_id || q.rdr_id!=1 || q.fcs_id!=1 || q.target_id>1) out.result=1;
        else if (!state.connected) out.result=2;
        else {
            uint16_t code;
            if (!q.target_id) {
                code=driver_.command(0xED06,0);
                if (code<=1) { stop_requested_=true;start_phase_=StartPhase::none; }
            } else if (start_phase_!=StartPhase::none || state.last_result==1) code=4;
            else {
                code=driver_.command(state.reporting?0xED06:0xED03,1);
                if (code<=1) {
                    stop_requested_=false;start_generation_=state.generation;
                    start_phase_=state.reporting?StartPhase::starting:StartPhase::reporting;
                }
            }
            out.result=code<=1?0:code==2?1:code==3||code==4?2:6;
        }
        return out;
    }
    c2::ipc::PipeResponse handle(const c2::ipc::PipeCommand& q,double now) {
        c2::ipc::PipeResponse out;out.request_id=q.request_id;out.command=q.command;
        out.result=5;
        if(!q.request_id || q.rdr_id!=1)out.result=1;
        else if(q.command==0)out.result=0; // RDR heartbeat; FCS status is handled by the station.
        else if(q.command==17)out.result=result(driver_.snapshot().last_result);
        else {
            uint16_t code=2;bool known=true;
            switch(q.command) {
            case 10:code=driver_.connect();break;
            case 11:code=driver_.command(0xED06,1);break;
            case 12:cancel_start();code=driver_.command(0xED06,0);break;
            case 13:code=driver_.command(0xED03,1);break;
            case 14:code=driver_.command(0xED03,0);break;
            case 15:case 16:
                if(q.target_id && q.target_id<=65535)code=driver_.command(0xED04,uint16_t(q.target_id),q.command==16?1:0);
                break;
            default:known=false;
            }
            out.result=known?result(code):5;
            if(known && (code==1 || (q.command==10 && code==0))) {
                command_=q.command;request_=q.request_id;previous_result_=code;
            }
            log_("GUI command="+std::to_string(q.command)+" target="+std::to_string(q.target_id)+" result="+std::to_string(out.result));
        }
        if(q.request_id && q.rdr_id==1){gui_seen_=now;stopped_=false;heartbeat_seen_=true;}
        const auto state=driver_.snapshot();out.connected=state.connected;
        if(q.command>=10) {
            out.fcs_id=command_;out.target_id=request_;out.assignment=state.mode;out.fire=state.reporting;
        }
        return out;
    }
    void populate(c2::ipc::Payload& p) {
        const auto state=driver_.snapshot();
        populate(p,state,c2::rdr::correct_targets(state,
            {p.rdr.position_x,p.rdr.position_y,p.rdr.position_z}));
    }
    void populate(c2::ipc::Payload& p, const c2::rdr::State& state,
                  const std::vector<c2::rdr::CorrectedTarget>& targets) {
        // This adapter owns the RDR and target fields. Keep the station's FCS
        // status, selection and installation positions when both peers run.
        p.targets={};p.target_count=0;
        p.rdr_count=p.selected_rdr=1;p.rdr.id=1;
        p.rdr.connected=p.rdr.enabled=state.connected;p.rdr.operating=operating(state);
        // RDR wire angles are signed tenths of a degree (confirmed device ICD).
        p.rdr.azimuth=state.pan==-32768?-32768:float(std::clamp(135+c2::rdr::heading_degrees(state.pan/10.),0.,270.));
        p.rdr.elevation=state.tilt==-32768?-32768:float(state.tilt/10.);
        for(const auto&t:targets) {
            if(p.target_count==c2::ipc::target_capacity)break;
            auto&v=p.targets[p.target_count++];v.id=t.id;v.rdr_id=1;
            v.valid=p.rdr.operating && t.valid;v.status=v.valid?0:t.status==3?3:2;
            if(v.valid) {
                // GUI XYZ is exclusively world coordinates, never sensor/relative XYZ.
                v.x=float(t.world.east);v.y=float(t.world.north);v.z=float(t.world.up);
                // Target table uses north=0 degrees without the map's display offset.
                v.azimuth=float(std::round(std::atan2(t.relative.east,t.relative.north)*180/std::numbers::pi*10)/10);
                v.elevation=float(std::round(std::atan2(t.relative.up,std::hypot(t.relative.east,t.relative.north))*180/std::numbers::pi*10)/10);
                v.distance=float(std::hypot(t.relative.east,t.relative.north,t.relative.up));
            }
        }
    }
};
