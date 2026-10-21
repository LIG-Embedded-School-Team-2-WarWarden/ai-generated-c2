#include "c2mvp/domain.hpp"
#include "target_detector.h"
#include <cmath>

namespace c2mvp {
std::vector<Object> LidarDetector::detect(unsigned width,unsigned height,const std::vector<pb::Point>& points){
    std::vector<Watch::Pixel> pixels;pixels.reserve(points.size());
    for(auto& p:points)pixels.push_back({p.xyz_valid()?p.x():0,p.xyz_valid()?p.y():0,p.xyz_valid()?p.z():0,p.xyz_valid()?p.sdk_depth_raw():0});
    Watch::Detector detector;std::vector<Object> objects;
    for(auto& t:detector.process(width,height,pixels).targets)objects.push_back({uid(),t.x,t.y,t.z,t.count});
    return objects;
}
Cloud::Cloud(Clock& c,PointDetector& d,bool threaded):clock(c),detector(d),threaded(threaded){if(threaded)worker=std::thread([this]{work();});}
Cloud::~Cloud(){ {std::lock_guard lock(mutex);closing=true;queued.reset();}cv.notify_all();if(worker.joinable())worker.join(); }
void Cloud::reset(){++generation;session.clear();high_frame=0;assembly.reset();sealed.clear();result.reset();state="NO_DATA";}
void Cloud::fail(const std::string& why){++generation;assembly.reset();sealed.clear();result.reset();state=why;}
bool Cloud::ingest(const std::string& connection,const pb::PointCloudChunk& c,bool verified){
    if(session!=connection){reset();session=connection;}
    if(!c.frame_id()||c.frame_id()<high_frame)return false;
    uint64_t n=uint64_t(c.width())*c.height();
    if(!n||n>500000||n!=c.total_points()||!c.chunk_count()||c.chunk_count()>8192||c.chunk_index()>=c.chunk_count()||!c.points_size()||c.motion_compensation()!=pb::PointCloudChunk::MOTION_COMPENSATION_NONE){fail("FRAME_INVALID");return false;}
    for(auto& p:c.points()){
        bool xyz=p.has_x()&&p.has_y()&&p.has_z();bool any=p.has_x()||p.has_y()||p.has_z();
        if(p.pixel_index()>=n||(p.xyz_valid()&&(!xyz||!finite(p.x())||!finite(p.y())||!finite(p.z())))||(!p.xyz_valid()&&any)||(p.has_intensity()&&(!finite(p.intensity())||!c.has_intensity_scale_id()))){fail("FRAME_INVALID");return false;}
    }
    auto meta=c;meta.clear_points();meta.set_chunk_index(0);
    if(c.frame_id()>high_frame){++generation;sealed.clear();high_frame=c.frame_id();state="UPDATING";result.reset();started=clock.monotonic();
        assembly=Assembly{meta,std::vector<pb::Point>(static_cast<size_t>(n)),std::vector<bool>(static_cast<size_t>(n)),{},0,verified};}
    const auto raw=c.SerializeAsString();
    if(!sealed.empty()){
        if(sealed.contains(c.chunk_index())&&sealed.at(c.chunk_index())==raw)return true;
        fail("FRAME_INVALID");return false;
    }
    if(!assembly||clock.monotonic()-started>=1){fail("FRAME_INCOMPLETE");return false;}
    auto& f=*assembly;
    if(meta.SerializeAsString()!=f.meta.SerializeAsString()||verified!=f.verified){fail("FRAME_INVALID");return false;}
    if(f.chunks.contains(c.chunk_index())){if(f.chunks.at(c.chunk_index())==raw)return true;fail("FRAME_INVALID");return false;}
    std::set<unsigned> indices;
    for(auto& p:c.points())if(!indices.insert(p.pixel_index()).second||f.occupied[p.pixel_index()]){fail("FRAME_INVALID");return false;}
    f.bytes+=raw.size();if(f.bytes>8*1024*1024){fail("FRAME_INVALID");return false;}
    f.chunks[c.chunk_index()]=raw;for(auto& p:c.points()){f.slots[p.pixel_index()]=p;f.occupied[p.pixel_index()]=true;}
    if(f.chunks.size()==c.chunk_count()){
        for(bool occupied:f.occupied)if(!occupied){fail("FRAME_INVALID");return false;}
        Job job{generation,session,f.meta,std::move(f.slots),verified};sealed=std::move(f.chunks);assembly.reset();state="PROCESSING";
        if(threaded){{std::lock_guard lock(mutex);queued=std::move(job);}cv.notify_one();}else execute(std::move(job));
    }
    return true;
}
void Cloud::tick(){if(assembly&&clock.monotonic()-started>=1)fail("FRAME_INCOMPLETE");}
void Cloud::execute(Job job){Completion c;c.job=std::move(job);try{c.objects=detector.detect(c.job.meta.width(),c.job.meta.height(),c.job.points);}catch(...){c.error=true;}
    std::lock_guard lock(mutex);done=std::move(c);}
void Cloud::work(){while(true){Job job;{std::unique_lock lock(mutex);cv.wait(lock,[&]{return closing||queued.has_value();});if(closing)return;job=std::move(*queued);queued.reset();}execute(std::move(job));}}
std::optional<Observation> Cloud::poll(){
    std::optional<Completion> c;{std::lock_guard lock(mutex);c=std::move(done);done.reset();}
    if(!c||c->job.generation!=generation||c->job.session!=session)return {};
    if(c->error){fail("DETECTOR_ERROR");return {};}
    return Observation{std::move(c->job.meta),std::move(c->objects),c->job.verified,clock.monotonic()};
}
ObservationService::ObservationService(Engine& h,PointDetector& d,bool worker):host(h),cloud(h.clock,d,worker){}
void ObservationService::invalidate(const std::string& why,bool reset){if(reset)cloud.reset();else cloud.fail(why);previous.reset();}
void ObservationService::trust_clock(const std::string& id,const Json& m){
    auto& p=host.devices.peers.at(id);
    require(m.at("boot_id")==p.boot&&m.at("connection_session_id")==p.session,"clock evidence is for another boot/session");
    double offset=m.at("offset_seconds"),unc=m.at("uncertainty_ms");
    require(!m.at("evidence").get<std::string>().empty()&&finite(offset)&&finite(unc)&&unc>=0&&unc<=host.config.uncertainty,"clock evidence/uncertainty required");
    p.mapping=m;
}
void ObservationService::trust_profile(const std::string& id,const Json& profile){
    auto& p=host.devices.peers.at(id);require(profile.at("sensor_profile")==host.config.devices.at(id).profile,"sensor profile mismatch");
    double start=stamp(profile.at("valid_from")),end=stamp(profile.at("valid_until"));
    require(start<=host.clock.time()&&host.clock.time()<end,"acquisition evidence expired/not active");
    auto mapping=profile;auto name=profile.at("id").get<std::string>();require(!name.empty()&&name.size()<=64,"invalid mapping ID");
    for(auto key:{"sync_type","sync_state","frame_protocol_version"})require(profile.at(key).is_number_integer()&&profile.at(key).get<int>()>=0&&profile.at(key).get<int>()<=255,"invalid SDK identifiers");
    require(profile.at("uncertainty_ms").get<double>()>=1,"measured uncertainty required");
    mapping["boot_id"]=p.boot;mapping["connection_session_id"]=p.session;mapping["valid_from"]=start;mapping["valid_until"]=end;trust_clock(id,mapping);
}
bool ObservationService::verify(const Peer& p,const pb::PointCloudChunk& c)const{
    auto& cfg=host.config.devices.at(p.id);
    require(c.installation_revision()==cfg.revision&&c.observation_mode_revision()==p.mode_revision&&c.sensor_data_profile_id()==cfg.profile,"frame installation/mode/profile mismatch");
    try{
        auto it=p.poses.find(c.pose_id());if(it==p.poses.end())return false;auto& pose=it->second;
        if(pose.obs_id()!=p.id||pose.collection_session_id()!=p.collection||pose.installation_revision()!=c.installation_revision()||pose.observation_mode_revision()!=p.mode_revision||!pose.pose_valid()||!pose.pan_measured()||!pose.tilt_measured()||!pose.has_pan_deg()||!pose.has_tilt_deg()||!finite(pose.pan_deg())||!finite(pose.tilt_deg())||std::abs(pose.pan_deg())>.001||std::abs(pose.tilt_deg())>.001||pose.motion_state()!=pb::MOTION_STATE_STOPPED||pose.captured_at()!=c.captured_at())return false;
        double start=stamp(c.captured_start_at()),at=stamp(c.captured_at()),end=stamp(c.captured_end_at()),now=host.clock.time();
        if(!(p.mode_applied<=start&&start<=at&&at<=end&&end<=now&&now-start<=1)||!c.has_sdk_time()||c.sdk_time().nanoseconds()>=1000000000)return false;
        Json m=p.mapping;
        if(host.config.simulation&&cfg.profile=="synthetic-organized-v1")m={{"id","synthetic-clock-v1"},{"offset_seconds",0},{"uncertainty_ms",1},{"sync_type",2},{"sync_state",1},{"frame_protocol_version",3}};
        if(m.is_null()||c.clock_mapping_id()!=m.at("id").get<std::string>())return false;
        if(m.contains("valid_until")&&!(m.at("valid_from").get<double>()<=start&&now<=m.at("valid_until").get<double>()))return false;
        auto& t=c.sdk_time();if(t.sync_type()!=m.at("sync_type").get<unsigned>()||t.sync_state()!=m.at("sync_state").get<unsigned>()||t.frame_protocol_version()!=m.at("frame_protocol_version").get<unsigned>())return false;
        if(!c.has_clock_uncertainty_ms()||!finite(c.clock_uncertainty_ms())||c.clock_uncertainty_ms()<m.at("uncertainty_ms").get<double>()||c.clock_uncertainty_ms()>host.config.uncertainty)return false;
        return std::abs(static_cast<double>(t.seconds())+t.nanoseconds()/1e9+m.at("offset_seconds").get<double>()-at)<=.0011;
    }catch(const std::exception&){return false;}
}
const Observation& ObservationService::latest()const{
    auto it=host.devices.peers.find(host.config.observer);
    if(it!=host.devices.peers.end()&&!it->second.mapping.is_null()&&it->second.mapping.contains("valid_until"))require(host.clock.time()<=it->second.mapping.at("valid_until").get<double>(),"acquisition evidence expired");
    require(cloud.state=="CURRENT"&&cloud.result&&cloud.result->verified,"no verified current observation");
    auto& r=*cloud.result;double age=host.clock.time()-stamp(r.meta.captured_start_at());
    require(host.clock.monotonic()-r.received<=1&&age>=0&&age<=1,"observation stale");return r;
}
void ObservationService::tick(){
    cloud.tick();auto r=cloud.poll();if(!r)return;
    if(previous&&r->objects.size()==1&&host.clock.monotonic()-previous->second<=1){auto& o=r->objects[0];auto& old=previous->first;
        if(std::hypot(std::hypot(o.x-old.x,o.y-old.y),o.z-old.z)<=.5)o.id=old.id;}
    previous.reset();if(r->objects.size()==1&&r->verified)previous=std::pair{r->objects[0],host.clock.monotonic()};
    cloud.state=r->verified?"CURRENT":"DISPLAY_ONLY";cloud.result=std::move(r);
    if(cloud.result->verified)host.effector.observed(cloud.result->objects,host.clock.monotonic());
}
}
