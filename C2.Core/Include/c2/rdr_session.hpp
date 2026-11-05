#pragma once
// RDR-ECS ICD revision 6: 24-byte controls, 32-byte float32 metre DE03.
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
using Packet=std::array<uint8_t,24>;
inline uint16_t word(std::span<const uint8_t> p,unsigned n){return uint16_t((unsigned(p[n])<<8)|p[n+1]);}
inline void word(Packet&p,unsigned n,uint16_t v){p.at(n)=uint8_t(v>>8);p.at(n+1)=uint8_t(v);}
inline uint32_t token(std::span<const uint8_t> p){return (uint32_t(word(p,10))<<16)|word(p,12);}
inline bool newer(uint16_t a,uint16_t b){auto d=uint16_t(a-b);return d&&d<32768;}
inline int signed_word(std::span<const uint8_t> p,unsigned n){const int v=word(p,n);return v>=32768?v-65536:v;}
struct Target {uint16_t id{};double x{},y{},z{},seen{};uint8_t status{2};};
struct State {
    uint64_t generation{}; // Local session lifetime; never serialized on the wire.
    bool connected{},reporting{};uint16_t mode{},motor{2};
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
    std::map<uint16_t,double> polls_;
    std::deque<uint16_t> issued_;
    std::vector<Packet> outgoing_;
    State state_;
    Packet make(uint16_t msg,uint16_t a=0,uint16_t b=0){
        Packet p{};word(p,0,msg);word(p,2,++sequence_);word(p,6,a);word(p,8,b);
        word(p,10,uint16_t(token_>>16));word(p,12,uint16_t(token_));
        issued_.push_back(sequence_);if(issued_.size()>256)issued_.pop_front();return p;
    }
    void invalidate(){for(auto& [id,t]:state_.targets){t.x=t.y=t.z=0;t.status=state_.excluded.count(id)?3:2;}}
    void fresh_session(double now){
        if(++token_==0)++token_;
        sequence_=uint16_t(token_^ (token_>>16));have_data_=have_status_=false;
        state_={};state_.generation=++generation_;state_.last_result=1;state_.last_command=0xED01;polls_.clear();issued_.clear();operation_.reset();outgoing_.clear();
        hello_=Pending{make(0xED01),now,now};outgoing_.push_back(hello_->packet);
    }
    bool status(std::span<const uint8_t> p,double now){
        const auto tx=word(p,2);if(have_status_&&!newer(tx,status_sequence_))return false;
        const auto flags=word(p,20);if((flags>>8)>4)return false;
        have_status_=true;status_sequence_=tx;state_.mode=flags>>8;state_.reporting=(flags&4)!=0;
        state_.pan=signed_word(p,18);state_.tilt=signed_word(p,16);
        if(word(p,0)==0xDE02)state_.motor=std::min<uint16_t>(2,word(p,6));
        if(!state_.reporting||state_.mode!=2){invalidate();have_data_=true;data_sequence_=tx;}
        last_status_=now;
        return true;
    }
public:
    explicit Session(uint32_t seed=1):token_(seed?seed:1){}
    const State& state()const{return state_;}
    std::vector<Packet> take(){auto out=std::move(outgoing_);outgoing_.clear();return out;}
    uint16_t connect(double now){wanted_=true;if(state_.connected)return 0;if(!hello_)fresh_session(now);return 1;}
    uint16_t command(uint16_t msg,uint16_t a,uint16_t b,double now){
        if(!state_.connected)return 3;
        if((msg!=0xED03&&msg!=0xED04&&msg!=0xED06)||
           ((msg==0xED03||msg==0xED06)&&a>1)||(msg==0xED04&&(!a||b>1)))return 2;
        // Stop can interrupt startup. Other mutations are serialized.
        if(operation_&&!(msg==0xED06&&a==0))return 4;
        operation_=Pending{make(msg,a,b),now,now};outgoing_.push_back(operation_->packet);
        state_.last_command=msg;state_.last_result=1;return 1;
    }
    void receive(std::span<const uint8_t> p,double now){
        if(p.size()<6)return;
        const auto msg=word(p,0),rx=word(p,4),tx=word(p,2);
        if(msg==0xDE03){
            if(p.size()!=32||!state_.connected||state_.mode!=2||!state_.reporting||!word(p,6)||word(p,24)>1)return;
            for(unsigned i=26;i<32;++i)if(p[i])return;
            auto coordinate=[&](unsigned n){return std::bit_cast<float>((uint32_t(word(p,n))<<16)|word(p,n+2));};
            const float x=coordinate(8),y=coordinate(12),z=coordinate(16);
            if(!std::isfinite(x)||!std::isfinite(y)||!std::isfinite(z))return;
            if(std::find(issued_.begin(),issued_.end(),rx)==issued_.end())return;
            if(have_data_&&!newer(tx,data_sequence_))return;
            have_data_=true;data_sequence_=tx;auto id=word(p,6);
            if(state_.excluded.count(id))return;
            if(!state_.targets.count(id)&&state_.targets.size()>=10)return;
            auto&t=state_.targets[id];t.id=id;t.seen=now;
            t.status=word(p,24)?0:2;
            t.x=t.status?0:x;t.y=t.status?0:y;t.z=t.status?0:z;
            return;
        }
        if(p.size()!=24||token(p)!=token_||word(p,14)>8||word(p,22))return;
        if(msg==0xDE01&&hello_&&rx==word(hello_->packet,2)){
            if(word(p,14)==0&&status(p,now)){state_.connected=true;state_.last_result=0;hello_.reset();next_poll_=now;}
            return;
        }
        if(!state_.connected)return;
        if(msg==0xDE02&&polls_.erase(rx)){
            if(word(p,14)==0&&status(p,now)){
                if(operation_&&word(operation_->packet,0)==0xED03&&
                   newer(rx,word(operation_->packet,2))&&state_.reporting==bool(word(operation_->packet,6))){
                    state_.last_result=0;operation_.reset();
                }
            }
            return;
        }
        if(!operation_||rx!=word(operation_->packet,2)||msg!=(0xDE00|(word(operation_->packet,0)&255)))return;
        const auto code=word(p,14);state_.last_result=code;status(p,now);
        if(code==1)return;
        if(code==0&&msg==0xDE04){
            const auto id=word(operation_->packet,6);
            if(word(operation_->packet,8))state_.excluded.erase(id);else state_.excluded.insert(id);
            if(state_.targets.count(id)||state_.targets.size()<10){
                auto&t=state_.targets[id];t.id=id;t.seen=now;t.x=t.y=t.z=0;t.status=state_.excluded.count(id)?3:2;
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
        if(operation_){
            if(now-operation_->born>40){operation_.reset();state_.last_result=9;}
            else if(now-operation_->last>=1){
                auto&p=*operation_;
                // ICD: retries preserve the original TxSeq and every packet byte.
                outgoing_.push_back(p.packet);p.last=now;
            }
        }
        if(now>=next_poll_){auto p=make(0xED02);polls_[word(p,2)]=now;outgoing_.push_back(p);next_poll_=now+1;}
        for(auto it=polls_.begin();it!=polls_.end();)if(now-it->second>4)it=polls_.erase(it);else ++it;
        for(auto it=state_.targets.begin();it!=state_.targets.end();){
            auto&t=it->second;
            if(now-t.seen>60&&!state_.excluded.count(it->first)){it=state_.targets.erase(it);continue;}
            if(now-t.seen>3&&t.status==0){t.status=2;t.x=t.y=t.z=0;}
            ++it;
        }
    }
};
}
