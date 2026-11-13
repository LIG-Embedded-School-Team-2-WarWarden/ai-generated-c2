#pragma once
#include "protocol.hpp"
#include "c2/fcs_client.hpp"
#include "c2/rdr_session.hpp"

// Non-blocking GUI start transaction. Never treats ED06 acceptance as optical readiness.
class RdrStartup {
public:
    using Clock=std::chrono::steady_clock;
    enum class Phase { idle, off, preparing, resuming, complete, failed };
    Phase phase{Phase::idle};
    c2::ipc::Result result{c2::ipc::Result::success};
    bool active() const {return phase==Phase::off||phase==Phase::preparing||phase==Phase::resuming;}
    bool blocks_recovery() const {return active()||phase==Phase::failed;}
    const char* name() const {
        static constexpr const char* names[]={"idle","fcs_off","rdr_preparing","lidar_resuming","complete","failed"};
        return names[unsigned(phase)];
    }
    void begin(const c2::ipc::PipeCommand& q,const c2::rdr::State& r,Clock::time_point now) {
        request_=q;generation_=r.generation;phase=Phase::off;
        result=c2::ipc::Result::status_pending;deadline_=now+std::chrono::seconds(10);
        off_ack_={};next_off_={};
    }
    template<class Control> void fail(Control& control,c2::ipc::Result why) {
        control.cancel_start();control.confirm_start(false);phase=Phase::failed;result=why;
    }
    template<class Driver,class Control,class Client>
    void tick(Driver& driver,Control& control,Client* client,Clock::time_point now) {
        if(!active())return;
        const auto r=driver.snapshot();
        using R=c2::ipc::Result;
        if(!r.connected||r.generation!=generation_){fail(control,R::lidar_disconnected);return;}
        if(now>=deadline_){fail(control,phase==Phase::off?R::fcs_off_unconfirmed:
            phase==Phase::preparing?R::rdr_start_unconfirmed:R::lidar_resume_unconfirmed);return;}
        if(phase==Phase::off) {
            const auto f=client?client->snapshot():c2::fcs::Snapshot{};
            // A disconnected/absent FCS does not gate RDR-only startup.
            if(f.connected && off_ack_==Clock::time_point{} && now>=next_off_) {
                next_off_=now+std::chrono::seconds(1);
                auto reply=client->laser(false);
                if(reply&&reply->result==c2::fcs::Result::accepted&&reply->laser_state==0)
                    off_ack_=Clock::now();
                return;
            }
            if(f.connected && (off_ack_==Clock::time_point{}||!f.status||f.status_received_at<=off_ack_||
                now-f.status_received_at>std::chrono::seconds(2)||!(f.status->flags&1)||(f.status->flags&30)))return;
            const auto reply=control.set_operating(request_);
            if(reply.result){fail(control,R::rdr_start_unconfirmed);return;}
            phase=Phase::preparing;deadline_=now+std::chrono::seconds(45);
        } else if(phase==Phase::preparing) {
            if(r.last_result>1){fail(control,R::rdr_start_unconfirmed);return;}
            if(r.mode!=2||!r.reporting||r.last_result!=0)return;
            // Recheck OFF if the FCS is connected, including a new connection during preparation.
            const auto f=client?client->snapshot():c2::fcs::Snapshot{};
            if(f.connected && (!f.status||now-f.status_received_at>std::chrono::seconds(2)||
                !(f.status->flags&1)||(f.status->flags&30))){fail(control,R::fcs_off_unconfirmed);return;}
            if(driver.command(0xED07,1)>1){fail(control,R::lidar_resume_unconfirmed);return;}
            phase=Phase::resuming;deadline_=now+std::chrono::duration_cast<Clock::duration>(
                std::chrono::duration<double>(c2::rdr::lidar_command_timeout_seconds));
        } else if(phase==Phase::resuming) {
            if(r.lidar_result>1){fail(control,R::lidar_resume_unconfirmed);return;}
            if(r.lidar_request==1&&r.lidar_result==0&&r.mode==2&&r.reporting&&
                r.lidar_supported&&r.lidar_known&&r.lidar_on&&!r.lidar_inhibited) {
                control.confirm_start(true);phase=Phase::complete;result=R::success;
            }
        }
    }
private:
    c2::ipc::PipeCommand request_{};
    uint64_t generation_{};
    Clock::time_point deadline_{},off_ack_{},next_off_{};
};
