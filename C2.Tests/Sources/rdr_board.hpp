#pragma once
#include <array>
#include <bit>
#include <cstdint>
#include <map>
#include <set>
#include <functional>
#include <stdexcept>
namespace Rdr {
using Packet=std::array<uint8_t,24>;
inline uint16_t get(const Packet&p,unsigned o){return uint16_t(p.at(o)*256u+p.at(o+1));}
template<size_t N> inline void put(std::array<uint8_t,N>&p,unsigned o,uint16_t v){p.at(o)=uint8_t(v>>8);p.at(o+1)=uint8_t(v);}
inline uint32_t session(const Packet&p){return uint32_t(get(p,10))*65536u+get(p,12);}
inline void session(Packet&p,uint32_t s){put(p,10,uint16_t(s>>16));put(p,12,uint16_t(s));}
inline bool newer(uint16_t a,uint16_t b){const auto d=uint16_t(a-b);return d!=0&&d<32768;}
enum Result:uint16_t {OK=0,PENDING=1,INVALID=2,NO_SESSION=3,BUSY=4,FAILED=5,UNKNOWN_ID=6,STALE=7,CONFLICT=8};
enum Mode:uint16_t {IDLE=0,STARTING=1,ACTIVE=2,STOPPING=3,FAULT=4};
class Engine {
    uint16_t tx_=0,last_=0; bool have_=false;
    std::map<uint16_t,std::pair<Packet,Packet>> cache_;
    Packet pending_{}; bool pending_valid_=false;
    Packet answer(const Packet&q,uint16_t code) {
        Packet p{};put(p,0,uint16_t(0xDE00|(get(q,0)&255)));put(p,2,++tx_);put(p,4,get(q,2));
        put(p,6,get(q,6));put(p,8,get(q,8));session(p,session(q));put(p,14,code);
        put(p,16,uint16_t(tilt));put(p,18,uint16_t(pan));
        put(p,20,uint16_t((connected?1:0)|(mode==ACTIVE?2:0)|(reporting?4:0)|(unsigned(mode)<<8)));
        if(get(q,0)==0xED02)put(p,6,motor);
        return p;
    }
    void reply(const Packet&q,uint16_t code){auto p=answer(q,code);cache_[get(q,2)]={q,p};if(cache_.size()>128)cache_.erase(cache_.begin());send(p);}
public:
    std::function<void(const Packet&)> send=[](const Packet&){};
    std::function<bool(bool)> control=[](bool){return false;};
    bool connected=false,reporting=false;uint32_t token=0;Mode mode=IDLE;
    double last_receive=0;uint16_t motor=2;int16_t pan=-32768,tilt=-32768;
    std::set<uint16_t> known,excluded;
    void receive(const Packet&q,double now){
        const auto msg=get(q,0),seq=get(q,2);const auto sid=session(q);
        if(msg!=0xED01&&msg!=0xED02&&msg!=0xED03&&msg!=0xED04&&msg!=0xED06)return;
        if(!sid)return;
        if(msg==0xED01){
            if(!connected){
                if(mode!=IDLE){send(answer(q,BUSY));return;}
                token=sid;connected=true;reporting=false;known.clear();excluded.clear();cache_.clear();have_=false;
            }else if(sid!=token){send(answer(q,BUSY));return;}
        }
        if(!connected||sid!=token){send(answer(q,NO_SESSION));return;}
        // Only valid, new commands (or exact retransmissions) renew the lease.
        auto old=cache_.find(seq);
        if(old!=cache_.end()){
            if(old->second.first==q){last_receive=now;if(msg!=0xED03)send(old->second.second);}
            else if(msg!=0xED03)send(answer(q,CONFLICT));
            return;
        }
        if(have_&&!newer(seq,last_)){if(msg!=0xED03)send(answer(q,STALE));return;}
        if((msg==0xED03||msg==0xED06)&&get(q,6)>1){if(msg!=0xED03)send(answer(q,INVALID));return;}
        if(msg==0xED04&&(!get(q,6)||get(q,8)>1)){reply(q,INVALID);return;}
        last_=seq;have_=true;last_receive=now;
        if(msg==0xED01||msg==0xED02){reply(q,OK);return;}
        if(msg==0xED03){reporting=get(q,6)!=0;cache_[seq]={q,{}};if(cache_.size()>128)cache_.erase(cache_.begin());return;}
        if(msg==0xED04){
            const auto id=get(q,6);if(!known.count(id)){reply(q,UNKNOWN_ID);return;}
            if(get(q,8))excluded.erase(id);else excluded.insert(id);reply(q,OK);return;
        }
        const bool start=get(q,6)!=0;
        if(mode==FAULT){reply(q,FAILED);return;}
        if((start&&mode==ACTIVE)||(!start&&mode==IDLE)){reply(q,OK);return;}
        if(start&&(mode==STARTING||mode==STOPPING||mode==FAULT)){reply(q,BUSY);return;}
        if(!start&&mode==STOPPING){reply(q,BUSY);return;}
        if(pending_valid_){reply(pending_,FAILED);pending_valid_=false;}
        mode=start?STARTING:STOPPING;pending_=q;pending_valid_=true;
        if(!control(start)){mode=FAULT;pending_valid_=false;reply(q,FAILED);return;}
        reply(q,PENDING);
    }
    void complete(bool started,bool ok){
        mode=ok?(started?ACTIVE:IDLE):FAULT;
        if(pending_valid_){reply(pending_,ok?OK:FAILED);pending_valid_=false;}
    }
    void fault(){mode=FAULT;reporting=false;if(pending_valid_){reply(pending_,FAILED);pending_valid_=false;}control(false);}
    void tick(double now){if(connected&&now-last_receive>3.5){
        connected=false;reporting=false;pending_valid_=false;
        if(mode!=IDLE){mode=STOPPING;if(!control(false))mode=FAULT;}
    }}
    std::function<void(const std::array<uint8_t,32>&)> send_observation=[](const auto&){};
    bool observation(std::array<uint8_t,32> p,uint16_t id){
        if(!id||!connected||mode!=ACTIVE)return false;
        known.insert(id);
        if(!reporting||excluded.count(id))return false;
        put(p,0,0xDE03);put(p,2,++tx_);put(p,4,last_);put(p,6,id);
        for(unsigned i=26;i<32;++i)p[i]=0;
        send_observation(p);return true;
    }
};
}
