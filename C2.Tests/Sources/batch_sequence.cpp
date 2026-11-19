#include "batch_sequence.hpp"
#include <source_location>
#include <stdexcept>
#include <string>

namespace {
using B=BatchSequence;using R=B::Result;using P=ManualSequence::Phase;
void check(bool ok,const std::source_location loc=std::source_location::current()) {
    if(!ok)throw std::runtime_error("batch sequence regression at line "+std::to_string(loc.line()));
}
// In-memory command recorder only. No sockets, device processes or simulator.
struct Fixture {
    B batch;
    c2::rdr::State r;
    c2::fcs::Snapshot f;
    InstallationConfig installation;
    std::vector<c2::rdr::CorrectedTarget> points;
    B::Clock::time_point now=B::Clock::now();
    double sample=100;
    bool legacy{},interlock_idle=true;
    int stops{},resumes{},captures{},moves{},offs{};
    R action_result=R::success;
    B::Actions actions;
    Fixture() {
        r.connected=r.reporting=r.lidar_supported=r.lidar_known=r.lidar_on=true;
        r.mode=2;r.generation=9;r.pan=1800;r.tilt=0;
        f.connected=true;f.status=c2::fcs::Status{1,0,180};f.status_received_at=now;
        for(uint16_t id=7;id<=16;++id) {
            c2::rdr::Target target;target.id=id;target.x=(id-7)*.2;target.z=3;target.y=.5;
            target.seen=sample;target.status=0;target.track_state=target.detected=1;target.pan=1800;target.tilt=0;
            r.targets[id]=target;
        }
        points=c2::rdr::correct_targets(r,{});
        actions.stop=[&]{++stops;interlock_idle=false;r.lidar_request=0;r.lidar_result=1;return action_result;};
        actions.resume=[&]{++resumes;r.lidar_request=1;r.lidar_result=1;return action_result;};
        actions.capture=[&]{++captures;return action_result;};
        actions.off=[&]{++offs;return action_result;};
        actions.clock=[&]{return now;};
        actions.point=[&](double tilt,double pan){++moves;f.status->azimuth=pan;f.status->elevation=tilt;return action_result;};
    }
    B::Context context() {
        return {r,f,points,installation,now,sample,true,legacy,interlock_idle};
    }
    R send(unsigned command,unsigned id=7) {return batch.command(command,id,context(),actions);}
    void advance(double seconds=.1) {
        now+=std::chrono::duration_cast<B::Clock::duration>(std::chrono::duration<double>(seconds));sample+=seconds;
        f.status_received_at=now;batch.tick(context());
    }
    void pause() {
        check(send(1204)==R::success);check(stops==1);
        check(!(batch.flags(context())&1));
        r.lidar_inhibited=true;r.lidar_on=false;r.lidar_result=0;
        advance();check(batch.flags(context())&1);
        // Retained GUI points become invalid and RDR ages its raw observations.
        for(auto& p:points)p.valid=false;
        r.targets.clear();
    }
    void capture(unsigned id) {
        check(send(1203,id)==R::success);check(batch.state(id)==110);
        check(send(1203,id)==R::success);
        advance();check(batch.state(id)==104);
        check(send(1205,id)==R::success);check(batch.state(id)==105);
        check(send(1205,id)==R::success);
        f.status->flags=13;advance();check(batch.state(id)==105);
        f.status->flags=1;advance();check(batch.state(id)==106);
    }
    void resume() {
        check(send(1206)==R::success);check(send(1206)==R::success);check(resumes==1);
        r.lidar_result=0;r.lidar_inhibited=false;r.lidar_on=true;interlock_idle=true;
        advance();
    }
};
}
int batch_sequence_entry() {
    { // GUI's literal 40-byte request/response contract, including all flag bits.
        for(unsigned code=1200;code<=1208;++code) {
            c2::ipc::PipeCommand q;q.command=code;q.request_id=12;q.rdr_id=q.fcs_id=1;q.target_id=7;
            c2::ipc::PipeCommand parsed;
            check(c2::ipc::deserialize(c2::ipc::serialize(q),parsed)&&parsed.command==code&&parsed.target_id==7);
            c2::ipc::PipeResponse p;p.command=code;p.request_id=12;p.target_id=7;p.assignment=112;p.active=15;
            c2::ipc::PipeResponse decoded;
            const auto bytes=c2::ipc::serialize(p);
            check(bytes.size()==40&&bytes[12]==(code&255)&&bytes[32]==15);
            check(c2::ipc::deserialize(bytes,decoded)&&decoded.active==15);
            p.active=16;check(!c2::ipc::deserialize(c2::ipc::serialize(p),decoded));
            p.command=0;p.active=2;check(!c2::ipc::deserialize(c2::ipc::serialize(p),decoded));
        }
    }
    { // GUI angle overrides are per row and become the alignment reference.
        Fixture x;
        check(x.send(1201)==R::success&&x.send(1201,8)==R::success);
        x.pause();
        const auto angles=gui_fcs_point_override(314.99,12.34,0);
        check(angles&&angles->azimuth==0&&angles->elevation==12.3);
        check(!gui_fcs_point_override(0,0,0));
        check(x.batch.command(1203,7,x.context(),x.actions,angles)==R::success);
        check(x.moves==1&&x.f.status->azimuth==0&&x.f.status->elevation==12.3);
        check(x.batch.command(1203,7,x.context(),x.actions,gui_fcs_point_override(130,20,0))==R::success);
        check(x.moves==1&&x.f.status->azimuth==0);
        x.advance();check(x.batch.state(7)==104);
        check(x.send(1202)==R::success);
        x.capture(8);
        check(x.f.status->azimuth!=0);
    }
    { // Two objects, one STOP, serial camera use, one resume and independent results.
        Fixture x;
        check(x.send(1200,0)==R::success);check(x.send(1200,99)==R::success&&!x.batch.state(99));
        check(x.send(1201)==R::success);check(x.send(1201)==R::success);
        check(x.send(1201,8)==R::success);check(x.batch.rows().size()==2&&x.moves==0&&x.captures==0);
        check(x.send(1205)==R::invalid_request);check(x.send(1203)==R::invalid_request);
        x.pause();check(x.send(1204)==R::success&&x.stops==1);
        check(x.send(1201,9)==R::device_busy); // no registration from a display snapshot
        check(x.send(1203)==R::success);check(x.send(1203,8)==R::device_busy);
        // An idle FE02 from before EF03 is not evidence the new move has ended.
        check(!(x.batch.flags(x.context())&2));check(x.send(1206)==R::status_pending);
        check(x.send(1202)==R::status_pending);
        x.advance();check(x.batch.state(7)==104);
        check(x.send(1205)==R::success);check(x.send(1205)==R::success&&x.captures==1);
        check(x.send(1202)==R::status_pending);check(x.send(1206)==R::status_pending);
        x.f.status->flags=13;x.advance();x.f.status->flags=1;x.advance();
        check(x.batch.state(7)==106&&(x.batch.flags(x.context())&4));
        x.advance(20);check(x.batch.state(7)==106&&x.resumes==0);
        x.capture(8);check(x.captures==2&&x.moves==2&&x.stops==1&&x.batch.state(7)==106);
        x.resume();check(x.batch.state(7)==107&&x.batch.state(8)==107);
        check(x.send(1202,8)==R::status_pending);
        c2::rdr::Target fresh;fresh.id=8;fresh.seen=x.sample+.01;fresh.status=0;
        fresh.track_state=fresh.detected=1;fresh.pan=1800;fresh.tilt=0;fresh.x=.2;fresh.z=3;fresh.y=.5;
        x.r.targets[8]=fresh;x.advance();check(x.batch.state(8)==111);
        x.advance(3);check(x.batch.state(7)==3&&x.batch.reason(7)==R::result_unconfirmed&&x.batch.state(8)==111&&x.batch.phase()==B::Phase::idle);
        x.points=c2::rdr::correct_targets(x.r,{});x.r.targets[8].seen=x.sample;
        check(x.send(1207,8)==R::success&&x.batch.state(8)==102);
        check(x.send(1207,8)==R::success);
        check(x.send(1202)==R::success&&!x.batch.state(7)&&x.batch.state(8)==102&&x.offs==0);
        check(x.send(1202,8)==R::success&&!x.batch.active()&&x.offs==1);
        check(x.send(1202,8)==R::success&&x.offs==1);
    }
    for(double arrival:{3.245,3.284069}) { // Boundary case and measured first-arrival delay.
        Fixture x;check(x.send(1201)==R::success);
        const auto original=x.r.targets.at(7);
        x.pause();x.capture(7);x.resume();x.advance(3);
        check(x.batch.state(7)==3&&x.batch.reason(7)==R::result_unconfirmed);
        check(x.batch.phase()==B::Phase::idle&&!x.batch.take_off_request());
        auto fresh=original;fresh.id=20;fresh.x+=.01;fresh.seen=x.sample+arrival-3;
        x.r.targets[20]=fresh;x.advance(arrival-3);check(x.batch.state(7)==3);
        x.advance(.1);check(x.batch.state(7)==3); // unchanged packet
        x.r.targets[20].seen=x.sample+1;x.advance(1);
        check(x.batch.state(7)==111&&x.batch.reason(7)==R::success);
        check(!x.r.targets.contains(7)&&x.r.targets.contains(20)); // no ID alias
        check(x.moves==1&&x.captures==1&&x.resumes==1&&x.offs==0);
        check(x.send(1207)==R::target_pending); // no implicit control reassignment
    }
    { // Stop completion, not acceptance or a stale optical sample, opens the slot.
        Fixture x;check(x.send(1201)==R::success);check(x.send(1204)==R::success);
        x.r.lidar_inhibited=true;x.r.lidar_on=false;x.r.lidar_known=false;x.r.lidar_result=0;
        x.advance(20);check(x.batch.phase()==B::Phase::stopping);
        check(x.send(1203)==R::invalid_request&&x.moves==0);
        x.advance(45);check(x.batch.state(7)==3&&x.batch.reason(7)==R::lidar_stop_unconfirmed);
        check(x.batch.take_off_request());check(!x.batch.take_off_request());
    }
    { // Lost target or predicted-only point cannot create a paused control snapshot.
        Fixture x;check(x.send(1201)==R::success);check(x.send(1201,8)==R::success);
        x.r.targets[8].track_state=2;
        check(x.send(1204)==R::target_pending&&x.stops==0);
        check(x.send(1202,8)==R::success);x.pause();
        x.r.generation++;x.advance();check(x.batch.state(7)==3&&x.batch.flags(x.context())==0);
        check(x.send(1203)==R::lidar_recovery_required&&x.moves==0);
    }
    { // Use the latest control point at STOP, not the registration-time position.
        Fixture x;check(x.send(1201)==R::success);
        x.points[0].world={1,3,.5};x.installation.fcs_angle_offset_deg={2,3,0};
        x.pause();const auto expected=fcs_point_angles(1,3,.5,2,3);
        check(x.send(1203)==R::success);
        check(x.f.status->azimuth==expected.azimuth&&x.f.status->elevation==expected.elevation);
    }
    { // Bound retained assignments and reject invalid or expired control sources.
        Fixture x;
        x.r.targets[7].seen=x.sample-4;check(x.send(1201)==R::target_pending);
        x.r.targets[7].seen=x.sample;
        for(unsigned id=7;id<=16;++id)check(x.send(1201,id)==R::success);
        check(x.send(1201,17)==R::already_assigned&&x.batch.rows().size()==10);
    }
    { // A failed/late resume cannot synthesize result observations.
        Fixture x;check(x.send(1201)==R::success);x.pause();x.capture(7);
        check(x.send(1206)==R::success);x.advance(65);
        check(x.batch.state(7)==3&&x.batch.reason(7)==R::lidar_resume_unconfirmed);
    }
    { // Expired status invalidates ownership even while no object is moving.
        Fixture x;check(x.send(1201)==R::success);x.pause();
        x.now+=std::chrono::seconds(3);x.batch.tick(x.context());
        check(x.batch.state(7)==3&&x.batch.reason(7)==R::status_pending);
    }
    { // Partial result/cancellation never stops another object's camera exposure.
        Fixture x;check(x.send(1201)==R::success);check(x.send(1201,8)==R::success);x.pause();
        x.capture(7);check(x.send(1203,8)==R::success);x.advance();check(x.send(1205,8)==R::success);
        check(x.send(1202)==R::success&&x.offs==0&&x.batch.state(8)==105);
        check(x.send(1202,8)==R::status_pending);
        x.f.status->flags=13;x.advance();x.f.status->flags=1;x.advance();
        check(x.send(1202,8)==R::success&&x.offs==1);
    }
    { // Operator may resume without capturing every registered object.
        Fixture x;check(x.send(1201)==R::success);check(x.send(1201,8)==R::success);x.pause();
        check(x.send(1203)==R::success);x.f.status->flags=3;
        check(x.send(1206)==R::status_pending&&x.resumes==0);
        x.f.status->flags=1;x.advance();check(x.batch.state(7)==104);
        x.resume();check(x.batch.state(7)==102&&x.batch.state(8)==102&&x.batch.phase()==B::Phase::idle);
        check(x.send(1203)==R::invalid_request);check(x.send(1204)==R::target_pending);
    }
    { // Early stop preserves the row and holds FCS until the return is confirmed.
        Fixture x;check(x.send(1201)==R::success);check(x.send(1201,8)==R::success);
        x.pause();check(x.send(1203)==R::success);x.advance();check(x.send(1205)==R::success);
        check(x.send(1208,8)==R::invalid_request&&x.offs==0);
        check(x.send(1208)==R::success&&x.batch.state(7)==112&&x.offs==1);
        check(x.send(1208)==R::success&&x.offs==1);
        x.batch.tick(x.context());check(x.batch.state(7)==112); // stale idle status
        x.f.status->flags=11;x.advance();check(x.batch.state(7)==112); // returning with light off
        check(x.send(1202)==R::status_pending);
        check(x.send(1206)==R::status_pending&&x.resumes==0);
        check(x.send(1203,8)==R::device_busy);
        x.f.status->flags=1;x.advance();
        check(x.batch.state(7)==106&&x.batch.state(8)==102&&x.batch.rows().size()==2);
        check(x.send(1208)==R::success&&x.offs==1&&x.resumes==0);
    }
    { // Confirmed capture is required; an ACK and idle FE02 are not completion.
        Fixture x;check(x.send(1201)==R::success);x.pause();
        check(x.send(1203)==R::success);x.advance();check(x.send(1205)==R::success);
        x.advance(30);check(x.batch.state(7)==3&&x.batch.reason(7)==R::fcs_off_unconfirmed);
    }
    { // An aligned object waits indefinitely, then captures only on operator input.
        Fixture x;check(x.send(1201)==R::success);x.pause();x.advance(60);
        check(x.batch.state(7)==102);check(x.send(1203)==R::success);x.advance();x.advance(10);
        x.advance(3600);
        check(x.batch.state(7)==104&&x.batch.reason(7)==R::success);
        check(x.captures==0&&x.resumes==0&&!x.batch.take_off_request());
        check(x.send(1205)==R::success&&x.captures==1&&x.batch.state(7)==105);
    }
    { // Cancellation remains available after a long operator wait.
        Fixture x;check(x.send(1201)==R::success);x.pause();
        check(x.send(1203)==R::success);x.advance();x.advance(3600);
        check(x.send(1202)==R::success&&!x.batch.active()&&x.offs==1);
    }
    for(auto failure:{R::response_timeout,R::device_rejected}) {
        Fixture x;check(x.send(1201)==R::success);x.pause();x.action_result=failure;
        check(x.send(1203)==failure&&x.batch.state(7)==3);
        check(x.send(1203)==R::lidar_recovery_required&&x.moves==1);
        check(x.send(1202)==failure&&x.batch.active()); // failed OFF must retain ownership
        x.action_result=R::success;check(x.send(1202)==R::success&&!x.batch.active());
    }
    { // Transport exceptions retain ownership and queue OFF, rather than retry motion.
        Fixture x;check(x.send(1201)==R::success);x.pause();
        x.actions.point=[](double,double)->R {throw std::runtime_error("IO failed");};
        bool threw=false;try{(void)x.send(1203);}catch(const std::runtime_error&){threw=true;}
        check(threw&&x.batch.state(7)==3&&x.batch.take_off_request());
    }
    { // Reproduce 12:04:18: STOP at PAN=1798/TILT=1002, then TILT=1003
      // during capture. One readback count must not fail all registered rows.
        Fixture x;x.r.pan=1798;x.r.tilt=1002;
        for(auto id:{7,8,9})check(x.send(1201,id)==R::success);
        x.pause();check(x.send(1203)==R::success);x.advance();
        check(x.send(1205)==R::success);x.f.status->flags=13;
        for(auto offset:{1,0,-1,1,0}) {
            x.r.pan=1798+offset;x.r.tilt=1002+offset;x.advance();
            check(x.batch.state(7)==105&&x.batch.state(8)==102&&x.batch.state(9)==102);
            check(!x.batch.take_off_request()&&x.offs==0);
        }
        x.f.status->flags=1;x.advance();check(x.batch.state(7)==106);
        x.capture(8);check(x.batch.state(7)==106&&x.batch.state(8)==106);
    }
    for(bool pan:{false,true})for(int sign:{-1,1}) {
        // Compare against the fixed reference, not against the previous sample.
        Fixture x;x.r.pan=1798;x.r.tilt=1002;
        check(x.send(1201)==R::success);x.pause();
        int& angle=pan?x.r.pan:x.r.tilt;
        angle+=sign;x.advance();check(x.batch.state(7)==102);
        angle+=sign;x.advance();
        check(x.batch.state(7)==3&&x.batch.reason(7)==R::target_changed&&x.batch.take_off_request());
    }
    for(auto initial:{0,3599}) {
        Fixture x;x.r.pan=initial;check(x.send(1201)==R::success);x.pause();
        x.r.pan=initial==0?3599:0;x.advance();check(x.batch.state(7)==102);
        x.r.pan=initial==0?3598:1;x.advance();check(x.batch.reason(7)==R::target_changed);
    }
    for(bool at_stop:{false,true})for(bool pan:{false,true}) {
        Fixture x;check(x.send(1201)==R::success);
        if(at_stop) {
            check(x.send(1204)==R::success);
            x.r.lidar_inhibited=true;x.r.lidar_on=false;x.r.lidar_result=0;
        } else x.pause();
        (pan?x.r.pan:x.r.tilt)=-32768;x.advance();
        check(x.batch.state(7)==3&&x.batch.reason(7)==R::target_changed&&x.batch.take_off_request());
    }
    for(int fault=0;fault<4;++fault) {
        Fixture x;check(x.send(1201)==R::success);x.pause();
        if(fault==0){x.r.sensor_state=9;x.r.sensor_health_valid=true;}
        if(fault==1)x.r.connected=false;
        if(fault==2)x.r.pan+=10;
        if(fault==3)x.f.status->flags=17;
        x.advance();check(x.batch.state(7)==3&&x.batch.flags(x.context())==0&&x.batch.take_off_request());
    }
    { // Fresh thermal warning plus completed STOP follows the existing optical policy.
        Fixture x;check(x.send(1201)==R::success);check(x.send(1204)==R::success);
        x.r.sensor_state=8;x.r.sensor_health_valid=true;x.r.lidar_known=x.r.lidar_on=false;
        x.r.lidar_inhibited=true;x.r.lidar_result=0;x.advance();check(x.batch.phase()==B::Phase::paused);
    }
    { // Session cannot steal legacy ownership; capability remains discoverable offline.
        Fixture x;x.legacy=true;check(x.send(1201)==R::device_busy&&x.batch.flags(x.context())==0);
        check(x.send(1200,0)==R::not_supported);
        x.legacy=false;x.f.connected=false;check(x.send(1200,0)==R::success);
        check(x.send(1201,0)==R::invalid_request);check(x.send(1201,65536)==R::invalid_request);
        x.legacy=false;check(x.send(1201)==R::status_pending);
    }
    return 0;
}
