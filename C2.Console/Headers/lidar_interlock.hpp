#pragma once
#include "c2/rdr_session.hpp"
#include "c2/fcs_client.hpp"

// Optical exclusion for the low-power demonstration laser. RDR ED07 owns a
// persistent inhibit latch; stopping reports or acquisition is not confirmation.
class LidarInterlock {
public:
    using Clock=std::chrono::steady_clock;
    enum class Phase { idle, stopping, held, restoring, fault };
    Phase phase() const { return phase_; }
    enum class Block { none, disconnected, unsupported, unknown, recovery_required, busy };
    Block block_reason(const c2::rdr::State& r) const {
        if(!r.connected)return Block::disconnected;
        if(!r.lidar_supported)return Block::unsupported;
        if(!r.lidar_known)return Block::unknown;
        if(phase_!=Phase::idle||r.lidar_inhibited)return Block::recovery_required;
        if(r.last_result==1)return Block::busy;
        return Block::none;
    }
    bool begin(const c2::rdr::State& r,Clock::time_point now) {
        if(block_reason(r)!=Block::none)return false;
        generation_=r.generation;phase_=Phase::stopping;started_=now;
        seen_emission_=off_confirmed_=false;return true;
    }
    bool stopped(const c2::rdr::State& r) const {
        return phase_==Phase::stopping&&r.connected&&r.generation==generation_&&
            r.lidar_request==0&&r.lidar_result==0&&r.lidar_supported&&
            r.lidar_known&&!r.lidar_on&&r.lidar_inhibited;
    }
    void emitted(Clock::time_point now) {phase_=Phase::held;started_=now;}
    void fail() {phase_=Phase::fault;off_confirmed_=false;}
    void off_reply(const std::optional<c2::fcs::Packet>& p,Clock::time_point now) {
        if(p&&p->result==c2::fcs::Result::accepted&&p->laser_state==0){
            phase_=Phase::held;off_confirmed_=true;started_=now;
        }
    }
    // True requests ED07=1 once. FE02 must be newer than the output/OFF command.
    bool update(const c2::rdr::State& r,const c2::fcs::Snapshot& f,Clock::time_point now) {
        if(phase_==Phase::restoring){
            if(!r.connected||r.generation!=generation_||now-started_>=std::chrono::duration<double>(c2::rdr::lidar_command_timeout_seconds)||
               r.lidar_result>1)fail();
            else if(r.lidar_request==1&&r.lidar_result==0&&r.lidar_supported&&
                    r.lidar_known&&!r.lidar_inhibited&&r.lidar_on==(r.mode==2))phase_=Phase::idle;
            return false;
        }
        if(phase_!=Phase::held)return false;
        if(!f.connected||!f.status||f.status_received_at<=started_||
           now-f.status_received_at>std::chrono::seconds(2))return false;
        const auto flags=f.status->flags;
        if(!(flags&1)||(flags&16))return false;
        if(flags&12){seen_emission_=true;return false;}
        if(!seen_emission_&&!off_confirmed_)return false;
        if(!r.connected||!r.lidar_supported||!r.lidar_known)return false;
        // Recovery after a reconnect requires an explicit acknowledged FCS OFF.
        if(r.generation!=generation_&&!off_confirmed_)return false;
        if(r.last_result==1)return false;
        if(!r.lidar_inhibited){phase_=Phase::idle;return false;}
        generation_=r.generation;phase_=Phase::restoring;started_=now;return true;
    }
private:
    Phase phase_{Phase::idle};
    uint64_t generation_{};
    Clock::time_point started_{};
    bool seen_emission_{},off_confirmed_{};
};
