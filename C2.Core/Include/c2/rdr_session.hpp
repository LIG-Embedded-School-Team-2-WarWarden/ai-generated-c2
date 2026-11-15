#pragma once
#include "diagnostic_log.hpp"
// RDR-ECS controls are 24 bytes. DE03 carries raw sensor XYZ, observation
// PAN/TILT and TrackState in a 32-byte big-endian packet.
#include <array>
#include <span>
#include <bit>
#include <cmath>
#include <cstdint>
#include <deque>
#include <map>
#include <optional>
#include <set>
#include <vector>
#include <algorithm>
namespace c2::rdr {
inline constexpr double lidar_command_timeout_seconds=6.0;
using Packet=std::array<uint8_t,24>;
inline uint16_t word(std::span<const uint8_t> p,unsigned n){return uint16_t((unsigned(p[n])<<8)|p[n+1]);}
inline void word(Packet&p,unsigned n,uint16_t v){p.at(n)=uint8_t(v>>8);p.at(n+1)=uint8_t(v);}
inline uint32_t token(std::span<const uint8_t> p){return (uint32_t(word(p,10))<<16)|word(p,12);}
inline bool newer(uint16_t a,uint16_t b){auto d=uint16_t(a-b);return d&&d<32768;}
inline int signed_word(std::span<const uint8_t> p,unsigned n){const int v=word(p,n);return v>=32768?v-65536:v;}
struct Target {
    uint16_t id{};
    double x{},y{},z{}; // Raw metres: mirrored X, up, forward; normalize X at world conversion.
    double seen{}; // Monotonic DE03 receive time: timeout and display sample identity.
    int pan{-32768},tilt{-32768}; // Observation pose, signed tenths of a degree.
    uint16_t detected{},track_state{}; // 0 LOST, 1 OBSERVED, 2 PREDICTED.
    uint8_t status{2};
};
struct State {
    uint64_t generation{}; // Local session lifetime; never serialized on the wire.
    bool connected{},reporting{};uint16_t mode{},motor{2};
    bool lidar_supported{},lidar_known{},lidar_on{},lidar_inhibited{};
    uint16_t lidar_request{},lidar_result{}; // ED07 completion survives unrelated commands.
    uint16_t sensor_state{255};bool sensor_health_valid{},thermal_protective_stop{};
    double sensor_health_at{};
    bool thermal_warning()const{return sensor_health_valid&&(sensor_state==7||sensor_state==8);}
    bool thermal_fault()const{return thermal_protective_stop||(sensor_health_valid&&sensor_state==9);}
    bool stop_completed()const{return connected&&lidar_supported&&lidar_inhibited&&
        lidar_request==0&&lidar_result==0&&!lidar_on&&!thermal_fault()&&
        ((lidar_known&&!lidar_on)||(!lidar_known&&thermal_warning()));}
    int pan{-32768},tilt{-32768};std::map<uint16_t,Target> targets;
    std::set<uint16_t> excluded;
    uint16_t last_result{},last_command{}; // 0..8 board result; 9 local timeout
};
class Session {
    struct Pending {Packet packet;double born,last;};
    uint32_t token_;uint16_t sequence_{},data_sequence_{},status_sequence_{};
    uint64_t generation_{};
    bool wanted_=false,have_data_=false,have_status_=false;
    double last_status_=0,next_poll_=0;
    std::optional<Pending> hello_,operation_;
    std::map<uint16_t,double> polls_,health_polls_;
    bool health_enabled_=false,have_health_=false;uint16_t health_sequence_{};
    std::deque<uint16_t> issued_;
    std::vector<Packet> outgoing_;
    State state_;
    void expire_operation(double now){
        if(!operation_)return;
        const bool lidar=word(operation_->packet,0)==0xED07;
        if(now-operation_->born>=(lidar?lidar_command_timeout_seconds:40.0)){
            diagnostic("RDR","command_timeout",{{"message_id",word(operation_->packet,0)},
                {"tx_seq",word(operation_->packet,2)},{"session",token_},{"elapsed_ms",(now-operation_->born)*1000}});
            if(lidar)state_.lidar_result=9;
            operation_.reset();state_.last_result=9;
        }
    }
    Packet make(uint16_t msg,uint16_t a=0,uint16_t b=0){
        Packet p{};word(p,0,msg);word(p,2,++sequence_);word(p,6,a);word(p,8,b);
        word(p,10,uint16_t(token_>>16));word(p,12,uint16_t(token_));
        issued_.push_back(sequence_);if(issued_.size()>256)issued_.pop_front();return p;
    }
    static void invalidate(Target& t,uint8_t status=2){
        t.x=t.y=t.z=0;t.pan=t.tilt=-32768;t.detected=t.track_state=0;t.status=status;
    }
    void invalidate(){for(auto& [id,t]:state_.targets)invalidate(t,state_.excluded.count(id)?3:2);}
    void fresh_session(double now){
        if(++token_==0)++token_;
        sequence_=uint16_t(token_^ (token_>>16));have_data_=have_status_=false;
        state_={};state_.generation=++generation_;state_.last_result=1;state_.last_command=0xED01;polls_.clear();health_polls_.clear();have_health_=false;issued_.clear();operation_.reset();outgoing_.clear();
        diagnostic("RDR","session_start",{{"session",token_},{"generation",generation_}});
        hello_=Pending{make(0xED01),now,now};outgoing_.push_back(hello_->packet);
    }
    bool status(std::span<const uint8_t> p,double now){
        const auto tx=word(p,2);if(have_status_&&!newer(tx,status_sequence_)){
            diagnostic("RDR","status_rejected",{{"reason","stale_sequence"},{"tx_seq",tx},{"previous_seq",status_sequence_}});return false;}
        const auto flags=word(p,20);if((flags>>8)>4){
            diagnostic("RDR","status_rejected",{{"reason","invalid_mode"},{"flags",flags},{"tx_seq",tx}});return false;}
        have_status_=true;status_sequence_=tx;state_.mode=flags>>8;state_.reporting=(flags&4)!=0;
        state_.lidar_supported=(flags&8)!=0;state_.lidar_known=(flags&16)!=0;
        state_.lidar_on=(flags&32)!=0;state_.lidar_inhibited=(flags&64)!=0;
        state_.thermal_protective_stop|=(flags&128)!=0;
        diagnostic("RDR","status_accepted",{{"message_id",word(p,0)},{"tx_seq",tx},{"reply_to",word(p,4)},
            {"session",token_},{"generation",generation_},{"flags",flags},{"mode",state_.mode},
            {"reporting",state_.reporting},{"lidar_supported",state_.lidar_supported},
            {"lidar_known",state_.lidar_known},{"lidar_on",state_.lidar_on},{"lidar_inhibited",state_.lidar_inhibited},
            {"result",word(p,14)}});
        state_.pan=signed_word(p,18);state_.tilt=signed_word(p,16);
        if(word(p,0)==0xDE02)state_.motor=std::min<uint16_t>(2,word(p,6));
        if(!state_.reporting||state_.mode!=2){invalidate();have_data_=true;data_sequence_=tx;}
        last_status_=now;
        return true;
    }
public:
    explicit Session(uint32_t seed=1):token_(seed?seed:1){}
    void enable_sensor_health(bool enabled=true){health_enabled_=enabled;}
    const State& state()const{return state_;}
    std::vector<Packet> take(){auto out=std::move(outgoing_);outgoing_.clear();return out;}
    uint16_t connect(double now){wanted_=true;if(state_.connected)return 0;if(!hello_)fresh_session(now);return 1;}
    uint16_t command(uint16_t msg,uint16_t a,uint16_t b,double now){
        if(!state_.connected)return 3;
        expire_operation(now);
        if((msg!=0xED03&&msg!=0xED04&&msg!=0xED06&&msg!=0xED07)||
           ((msg==0xED03||msg==0xED06||msg==0xED07)&&a>1)||(msg==0xED04&&(!a||b>1))||
           (msg==0xED07&&b))return 2;
        if(msg==0xED07&&!state_.lidar_supported)return 2;
        // Stop can interrupt startup. Other mutations are serialized.
        if(operation_&&!(msg==0xED06&&a==0))return 4;
        operation_=Pending{make(msg,a,b),now,now};outgoing_.push_back(operation_->packet);
        if(msg==0xED07){state_.lidar_request=a;state_.lidar_result=1;}
        state_.last_command=msg;state_.last_result=1;return 1;
    }
    void receive(std::span<const uint8_t> p,double now){
        auto reject=[&](const char* reason){
            diagnostic("RDR","rx_rejected",{{"reason",reason},{"session",token_},{"length",p.size()},
                {"message_id",p.size()>=2?word(p,0):0},{"tx_seq",p.size()>=4?word(p,2):0},
                {"reply_to",p.size()>=6?word(p,4):0}});
        };
        if(p.size()<6){reject("short_header");return;}
        const auto msg=word(p,0),rx=word(p,4),tx=word(p,2);
        if(msg==0xDE03){
            if(p.size()!=32||!state_.connected||state_.mode!=2||!state_.reporting||!word(p,6)){reject("target_length_or_state_or_id");return;}
            for(unsigned i=28;i<32;++i)if(p[i]){reject("target_padding");return;}
            const auto detected=word(p,24),track_state=word(p,26);
            if(track_state>2||detected!=(track_state==1?1:0)){reject("track_state_detected_mismatch");return;}
            auto coordinate=[&](unsigned n){return std::bit_cast<float>((uint32_t(word(p,n))<<16)|word(p,n+2));};
            const float x=coordinate(8),y=coordinate(12),z=coordinate(16);
            if(!std::isfinite(x)||!std::isfinite(y)||!std::isfinite(z)){reject("nonfinite_xyz");return;}
            const int tilt=signed_word(p,20),pan=signed_word(p,22);
            if(track_state==0&&(x!=0||y!=0||z!=0||tilt!=-32768||pan!=-32768)){reject("invalid_lost_payload");return;}
            if(std::find(issued_.begin(),issued_.end(),rx)==issued_.end()){reject("unknown_reply_to");return;}
            if(have_data_&&!newer(tx,data_sequence_)){reject("stale_target_sequence");return;}
            have_data_=true;data_sequence_=tx;auto id=word(p,6);
            if(state_.excluded.count(id)){reject("excluded_target");return;}
            if(!state_.targets.count(id)&&state_.targets.size()>=10){
                // A full set of expired tracks must not reject newly observed
                // identities for the entire 60 s diagnostic retention period.
                if(track_state!=1){reject("target_capacity_predicted");return;}
                auto victim=state_.targets.end();
                for(auto it=state_.targets.begin();it!=state_.targets.end();++it)
                    if(!state_.excluded.count(it->first)&&(it->second.status==2||now-it->second.seen>3)&&
                       (victim==state_.targets.end()||it->second.seen<victim->second.seen))victim=it;
                if(victim==state_.targets.end()){reject("target_capacity");return;}
                state_.targets.erase(victim);
            }
            auto&t=state_.targets[id];t.id=id;t.seen=now;
            t.detected=detected;t.track_state=track_state;
            t.status=track_state?0:2;
            t.x=x;t.y=y;t.z=z;t.pan=pan;t.tilt=tilt;
            return;
        }
        if(p.size()!=24){reject("control_length");return;}
        if(token(p)!=token_){reject("session_mismatch");return;}
        if(word(p,14)>8||word(p,22)){reject("control_result_or_padding");return;}
        expire_operation(now); // A late completion cannot revive an expired request.
        if(msg==0xDE01&&hello_&&rx==word(hello_->packet,2)){
            if(word(p,14)==0&&status(p,now)){state_.connected=true;state_.last_result=0;hello_.reset();next_poll_=now;}
            return;
        }
        if(!state_.connected){reject("not_connected");return;}
        if(msg==0xDE08){
            auto it=health_polls_.find(rx);
            if(it==health_polls_.end()||now-it->second>2.5){reject("health_unmatched_or_expired");return;}
            if(word(p,14)||word(p,6)>255||word(p,8)>3){reject("health_invalid_fields");return;}
            if(have_health_&&!newer(tx,health_sequence_)){reject("health_stale_sequence");return;}
            health_polls_.erase(it);have_health_=true;health_sequence_=tx;
            state_.sensor_health_valid=(word(p,8)&1)&&word(p,6)!=255;
            state_.sensor_state=state_.sensor_health_valid?word(p,6):255;
            state_.thermal_protective_stop|=(word(p,8)&2)!=0;
            state_.sensor_health_at=now;
            diagnostic("RDR","sensor_health",{{"state",state_.sensor_state},{"valid",state_.sensor_health_valid},
                {"protective_stop",state_.thermal_protective_stop},{"session",token_},{"tx_seq",tx},{"reply_to",rx}});
            return;
        }
        if(msg==0xDE02&&polls_.erase(rx)){
            if(word(p,14)==0&&status(p,now)){
                if(operation_&&word(operation_->packet,0)==0xED03&&
                   newer(rx,word(operation_->packet,2))&&state_.reporting==bool(word(operation_->packet,6))){
                    state_.last_result=0;operation_.reset();
                }
            }
            return;
        }
        if(!operation_||rx!=word(operation_->packet,2)||msg!=(0xDE00|(word(operation_->packet,0)&255))){reject("no_matching_operation");return;}
        if(msg==0xDE07&&(word(p,6)!=word(operation_->packet,6)||word(p,8))){reject("lidar_echo_mismatch");return;}
        const auto code=word(p,14);
        if(msg==0xDE07){
            const auto flags=word(p,20);
            if(code==0&&(!(flags&8)||
                (word(p,6)==0?(!(flags&64)||(flags&32)):
                    ((flags&64)||((flags&16)&&bool(flags&32)!=((flags>>8)==2)))))){reject("lidar_completion_flags");return;}
            // A cached final reply may precede a newer DE02. Never roll status
            // back, but accept its matching completion if current status agrees.
            if(!status(p,now)&&(code!=0||!state_.lidar_supported||
                (word(p,6)==0?(!state_.lidar_inhibited||(state_.lidar_known&&state_.lidar_on)):
                    (state_.lidar_inhibited||(state_.lidar_known&&state_.lidar_on!=(state_.mode==2)))))){reject("stale_lidar_completion_conflict");return;}
        }else status(p,now);
        diagnostic("RDR","command_reply",{{"message_id",msg},{"tx_seq",tx},{"reply_to",rx},{"session",token_},
            {"result",code},{"elapsed_ms",(now-operation_->born)*1000}});
        state_.last_result=code;
        if(msg==0xDE07)state_.lidar_result=code;
        if(code==1)return;
        if(code==0&&msg==0xDE04){
            const auto id=word(operation_->packet,6);
            if(word(operation_->packet,8))state_.excluded.erase(id);else state_.excluded.insert(id);
            if(state_.targets.count(id)||state_.targets.size()<10){
                auto&t=state_.targets[id];t.id=id;t.seen=now;invalidate(t,state_.excluded.count(id)?3:2);
            }
        }
        operation_.reset();
    }
    void tick(double now){
        if(state_.connected&&now-last_status_>3.2){fresh_session(now);}
        if(!state_.connected){
            if(wanted_&&!hello_)fresh_session(now);
            if(hello_&&now-hello_->last>=1){outgoing_.push_back(hello_->packet);hello_->last=now;}
            return;
        }
        expire_operation(now);
        if(now-state_.sensor_health_at>=2.5){state_.sensor_health_valid=false;state_.sensor_state=255;}
        if(operation_){
            if(now-operation_->last>=1){
                auto&p=*operation_;
                // ICD: retries preserve the original TxSeq and every packet byte.
                outgoing_.push_back(p.packet);p.last=now;
            }
        }
        if(now>=next_poll_){auto p=make(0xED02);polls_[word(p,2)]=now;outgoing_.push_back(p);next_poll_=now+1;
            if(health_enabled_){auto h=make(0xED08);health_polls_[word(h,2)]=now;outgoing_.push_back(h);}}
        for(auto it=health_polls_.begin();it!=health_polls_.end();)if(now-it->second>2.5)it=health_polls_.erase(it);else ++it;
        for(auto it=polls_.begin();it!=polls_.end();)if(now-it->second>4)it=polls_.erase(it);else ++it;
        for(auto it=state_.targets.begin();it!=state_.targets.end();){
            auto&t=it->second;
            if(now-t.seen>60&&!state_.excluded.count(it->first)){it=state_.targets.erase(it);continue;}
            // Sender owns the 1.5 s prediction limit from its last observation
            // or point-cloud support. DE03 has no observation timestamp, so
            // receipt of a prediction must not be treated as a new observation.
            // This independent 3 s timeout only detects missing DE03 traffic.
            if(now-t.seen>3&&t.status==0)invalidate(t);
            ++it;
        }
    }
};
}
