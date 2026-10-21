#include "c2mvp/domain.hpp"
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/io/zero_copy_stream_impl_lite.h>
#include <chrono>
#include <cmath>
#include <ctime>
#include <iomanip>
#include <random>
#include <regex>
#include <sstream>
#include <numbers>

namespace c2mvp {
void require(bool ok,const std::string& why){if(!ok)throw std::invalid_argument(why);}
bool finite(double v){return std::isfinite(v);}
std::string uid(){
    thread_local std::mt19937_64 rng(std::random_device{}());
    unsigned char b[16];for(auto& x:b)x=static_cast<unsigned char>(rng());
    b[6]=(b[6]&15)|64;b[8]=(b[8]&63)|128;
    std::ostringstream s;s<<std::hex<<std::setfill('0');
    for(int i=0;i<16;++i){if(i==4||i==6||i==8||i==10)s<<'-';s<<std::setw(2)<<int(b[i]);}return s.str();
}
double SystemClock::time()const{return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();}
double SystemClock::monotonic()const{return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}
std::string utc(double value){
    auto millis=static_cast<int64_t>(std::floor(value*1000)); auto seconds=static_cast<time_t>(millis/1000);
    std::tm tm{};
#ifdef _WIN32
    gmtime_s(&tm,&seconds);
#else
    gmtime_r(&seconds,&tm);
#endif
    std::ostringstream out;out<<std::put_time(&tm,"%Y-%m-%dT%H:%M:%S")<<'.'<<std::setw(3)<<std::setfill('0')<<millis%1000<<'Z';return out.str();
}
double stamp(const std::string& s){
    static const std::regex re(R"(\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}\.\d{3}Z)");
    require(std::regex_match(s,re),"UTC milliseconds required");
    std::tm tm{}; std::istringstream in(s.substr(0,19));in>>std::get_time(&tm,"%Y-%m-%dT%H:%M:%S");
    require(!in.fail(),"invalid timestamp");
#ifdef _WIN32
    auto secs=_mkgmtime(&tm);
#else
    auto secs=timegm(&tm);
#endif
    double v=static_cast<double>(secs)+std::stoi(s.substr(20,3))/1000.0;
    require(utc(v)==s,"invalid calendar timestamp");return v;
}
static const google::protobuf::Message& body(const pb::Envelope& e){
    auto* f=e.GetReflection()->GetOneofFieldDescriptor(e,e.GetDescriptor()->FindOneofByName("body"));
    require(f!=nullptr,"missing body");return e.GetReflection()->GetMessage(e,f);
}
static std::string string_field(const google::protobuf::Message& m,const char* name){
    auto* f=m.GetDescriptor()->FindFieldByName(name);return f?m.GetReflection()->GetString(m,f):std::string{};
}
std::string request_id(const pb::Envelope& e){return string_field(body(e),"request_id");}
const pb::Result* outcome(const pb::Envelope& e){
    auto& b=body(e);auto* f=b.GetDescriptor()->FindFieldByName("outcome");
    return f?dynamic_cast<const pb::Result*>(&b.GetReflection()->GetMessage(b,f)):nullptr;
}
pb::Envelope envelope(pb::Envelope e,const std::string& peer,const std::string& session,const Clock& c){
    e.set_version_major(1);e.set_version_minor(0);e.set_sender_id("C2");e.set_recipient_id(peer);
    if(!session.empty())e.set_connection_session_id(session);
    e.set_message_id(uid());e.set_sent_at(c.utc());return e;
}
std::string encode(const pb::Envelope& e){
    require(e.ByteSizeLong()<=1200,"Envelope exceeds 1200 bytes");
    std::string s;google::protobuf::io::StringOutputStream raw(&s);
    {google::protobuf::io::CodedOutputStream stream(&raw);stream.SetSerializationDeterministic(true);require(e.SerializeToCodedStream(&stream),"encode failed");}return s;
}
pb::Envelope decode(const std::string& raw){
    require(raw.size()<=1200,"oversized UDP Envelope");pb::Envelope e;
    require(e.ParseFromString(raw),"invalid protobuf");
    require(e.version_major()==1&&e.version_minor()==0&&e.body_case()!=pb::Envelope::BODY_NOT_SET,"unsupported version/body");
    require(!e.sender_id().empty()&&e.recipient_id()=="C2","invalid sender/recipient");
    static const std::regex uuid(R"([0-9a-fA-F]{8}-[0-9a-fA-F]{4}-[0-9a-fA-F]{4}-[0-9a-fA-F]{4}-[0-9a-fA-F]{12})");
    require(std::regex_match(e.message_id(),uuid),"invalid message UUID");stamp(e.sent_at());return e;
}
Config Config::parse(const Json& j){
    Config c;c.simulation=j.value("simulation",false);c.observer=j.at("observer");c.uncertainty=j.at("max_clock_uncertainty_ms");
    require(finite(c.uncertainty)&&c.uncertainty>0,"clock uncertainty required");
    for(auto& [id,v]:j.at("devices").items()){
        Installation i;i.role=v.at("role");i.revision=v.at("revision");i.calibration=v.at("calibration_id");
        i.profile=v.value("profile","");i.position=v.at("position").get<std::array<double,3>>();
        i.offset=v.at("sensor_offset").get<std::array<double,3>>();i.rotation=v.at("rotation").get<std::array<double,9>>();
        require(i.revision>0&&!i.calibration.empty()&&(i.role=="OBS"||i.role=="EFF"),"invalid installation");
        for(auto x:i.position)require(finite(x),"invalid position");for(auto x:i.offset)require(finite(x),"invalid offset");
        auto& r=i.rotation;for(auto x:r)require(finite(x),"invalid rotation");
        for(int a=0;a<3;++a)for(int b=0;b<3;++b){double dot=0;for(int k=0;k<3;++k)dot+=r[a*3+k]*r[b*3+k];require(std::abs(dot-(a==b))<1e-6,"rotation must be orthonormal");}
        double det=r[0]*(r[4]*r[8]-r[5]*r[7])-r[1]*(r[3]*r[8]-r[5]*r[6])+r[2]*(r[3]*r[7]-r[4]*r[6]);
        require(std::abs(det-1)<1e-6,"rotation must be right handed");c.devices[id]=i;
    }
    require(c.devices.contains(c.observer)&&c.devices.at(c.observer).role=="OBS","observer missing");return c;
}
std::pair<double,double> RelativePointingSolver::solve(const Object& o,const Installation& obs,const Installation& eff)const{
    std::array<double,3> p{o.x+obs.offset[0],o.y+obs.offset[1],o.z+obs.offset[2]},delta{},local{};
    for(int i=0;i<3;++i){delta[i]=obs.position[i]-eff.position[i];for(int k=0;k<3;++k)delta[i]+=obs.rotation[3*i+k]*p[k];}
    for(int i=0;i<3;++i)for(int k=0;k<3;++k)local[i]+=eff.rotation[3*k+i]*delta[k];
    double h=std::hypot(local[0],local[1]);require(h>=1e-8&&finite(h)&&finite(local[2]),"undefined pointing angle");
    return {std::atan2(local[1],local[0])*180/std::numbers::pi,std::atan2(local[2],h)*180/std::numbers::pi};
}
std::string RequestTracker::send(const Peer& p,const pb::Envelope& cmd){auto bytes=encode(envelope(cmd,p.id,p.session,clock));transport.send(bytes,p.address);return bytes;}
void RequestTracker::request(const Peer& p,const pb::Envelope& cmd,pb::Envelope::BodyCase kind,Callback cb,double timeout){
    auto key=request_id(cmd);require(!key.empty()&&!pending.contains(key),"duplicate/empty request ID");
    // Generated C++ messages are copied by value: caller mutation cannot alter correlation.
    Pending v;v.peer=p;v.command=cmd;v.kind=kind;v.callback=std::move(cb);v.timeout=timeout;
    v.packet=send(p,v.command);v.started=v.sent=clock.monotonic();pending.emplace(key,std::move(v));
}
bool RequestTracker::has(const std::string& id,pb::Envelope::BodyCase kind)const{
    for(auto& [k,v]:pending)if(v.peer.id==id&&(kind==pb::Envelope::BODY_NOT_SET||v.kind==kind))return true;return false;
}
void RequestTracker::cancel(const std::string& id,std::set<pb::Envelope::BodyCase> kinds){
    std::erase_if(pending,[&](auto& kv){return kv.second.peer.id==id&&(kinds.empty()||kinds.contains(kv.second.kind));});
}
void RequestTracker::result(const Peer& p,const pb::Envelope& e){
    auto* out=outcome(e);if(!out)return;auto it=pending.find(out->request_id());if(it==pending.end())return;
    auto& v=it->second;if(v.peer.id!=p.id||v.peer.session!=p.session||v.kind!=e.body_case())return;
    const auto& q=v.command;
    if(e.has_control_result()){auto& r=e.control_result();auto& b=q.control_open();if(r.active_control_session_id()!=b.control_session_id()||r.control_epoch()!=b.control_epoch())return;}
    if(e.has_pointing_result()){auto& r=e.pointing_result();auto& b=q.pointing_command();if(r.control_session_id()!=b.control_session_id()||r.control_epoch()!=b.control_epoch()||r.command_sequence()!=b.command_sequence())return;}
    if(e.has_obs_motion_result()){auto& r=e.obs_motion_result();auto& b=q.obs_motion_command();if(r.obs_id()!=p.id||r.obs_control_session_id()!=b.obs_control_session_id()||r.control_epoch()!=b.control_epoch()||r.command_sequence()!=b.command_sequence())return;}
    if(out->has_detail()&&out->detail().size()>256)return;stamp(out->reported_at());
    if(out->result()==pb::REQUEST_RESULT_ACCEPTED){v.accepted=true;return;}
    if(out->result()<pb::REQUEST_RESULT_COMPLETED||out->result()>pb::REQUEST_RESULT_CANCELLED)return;
    if(!out->has_completed_at())return;stamp(out->completed_at());
    auto cb=std::move(v.callback);pending.erase(it);cb(&e);
}
void RequestTracker::tick(const std::map<std::string,Peer>& peers){
    std::vector<std::string> keys;for(auto& [key,v]:pending)keys.push_back(key);
    for(auto& key:keys){auto it=pending.find(key);if(it==pending.end())continue;auto& v=it->second;
        auto p=peers.find(v.peer.id);double now=clock.monotonic();
        if(p==peers.end()||p->second.session!=v.peer.session||now-v.started>=v.timeout||(!v.accepted&&now-v.sent>=.2&&v.attempts>=3)){
            auto cb=std::move(v.callback);pending.erase(it);cb(nullptr);
        }else if(!v.accepted&&now-v.sent>=.2){transport.send(v.packet,v.peer.address);v.sent=now;++v.attempts;}
    }
}
std::string name(ObserverPhase p){static const char* names[]={"WAIT_EFF_STOP","WAIT_COLLECTION_STOP","OPENING","MOVING","STOPPING","COLLECTING","COMPLETED","FAILED"};return names[static_cast<int>(p)];}
std::string name(EffectorPhase p){static const char* names[]={"OPENING","RUNNING","MANUAL","STOPPING","STOPPED"};return names[static_cast<int>(p)];}
bool ObserverOperation::terminal()const{return phase==ObserverPhase::Completed||phase==ObserverPhase::Failed;}
void ObserverOperation::transition(ObserverPhase next){
    if(next==phase)return;
    static const std::map<ObserverPhase,std::set<ObserverPhase>> allowed={
        {ObserverPhase::WaitEffStop,{ObserverPhase::WaitCollectionStop,ObserverPhase::Failed}},
        {ObserverPhase::WaitCollectionStop,{ObserverPhase::Opening,ObserverPhase::Completed,ObserverPhase::Failed}},
        {ObserverPhase::Opening,{ObserverPhase::Moving,ObserverPhase::Failed}},
        {ObserverPhase::Moving,{ObserverPhase::Collecting,ObserverPhase::Stopping,ObserverPhase::Failed}},
        {ObserverPhase::Stopping,{ObserverPhase::WaitCollectionStop,ObserverPhase::Failed}},
        {ObserverPhase::Collecting,{ObserverPhase::Completed,ObserverPhase::Failed}},
        {ObserverPhase::Completed,{}},{ObserverPhase::Failed,{}}};
    require(allowed.at(phase).contains(next),"illegal OBS transition: "+name(phase)+" -> "+name(next));phase=next;
}
}
