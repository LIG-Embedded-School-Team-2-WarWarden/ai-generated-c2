#include "manual_sequence.hpp"
#include <stdexcept>
#include <source_location>
int manual_sequence_entry() {
    using M=ManualSequence;using P=M::Phase;using R=M::Result;
    auto check=[](bool ok,const std::source_location loc=std::source_location::current()){if(!ok)throw std::runtime_error("manual/thermal regression at line "+std::to_string(loc.line()));};
    const auto start=M::Clock::now(),later=start+std::chrono::hours(1);
    { // Device STOP is bounded; operator waiting after confirmed STOP is not.
        M waiting;waiting.generation=1;waiting.paused_at=start;waiting.enter(P::pausing,start);
        c2::rdr::State pending;pending.connected=pending.reporting=pending.lidar_supported=true;
        pending.mode=2;pending.generation=1;pending.lidar_result=1;
        c2::fcs::Snapshot idle;idle.connected=true;idle.status=c2::fcs::Status{1,0,0};
        idle.status_received_at=start+std::chrono::seconds(20);
        waiting.tick(pending,idle,true,idle.status_received_at,20);
        check(waiting.phase==P::pausing);
        pending.lidar_result=0;pending.lidar_known=pending.lidar_inhibited=true;
        waiting.tick(pending,idle,true,idle.status_received_at,20);
        check(waiting.phase==P::paused&&waiting.paused_at==idle.status_received_at);
        idle.status_received_at=start+std::chrono::seconds(30);
        waiting.tick(pending,idle,true,idle.status_received_at,30);
        check(waiting.phase==P::paused&&waiting.reason==R::success);
    }
    c2::rdr::State r;r.connected=r.reporting=r.lidar_supported=r.lidar_known=r.lidar_inhibited=true;
    r.mode=2;r.generation=1;
    { // A late operator request still gets a full 30 s completion window.
        M capture;capture.generation=1;capture.paused_at=start;
        const auto requested=start+std::chrono::seconds(9);
        capture.enter(P::firing,requested);
        c2::fcs::Snapshot busy;busy.connected=true;busy.status=c2::fcs::Status{13,0,0};
        for(auto elapsed:{1,8,10,29}) {
            busy.status_received_at=requested+std::chrono::seconds(elapsed);
            capture.tick(r,busy,true,busy.status_received_at,elapsed);
            check(capture.phase==P::firing&&capture.output_seen);
        }
        auto completed=capture;
        busy.status_received_at=requested+std::chrono::seconds(30);
        capture.tick(r,busy,true,busy.status_received_at,30);
        check(capture.phase==P::failed&&capture.reason==R::fcs_off_unconfirmed);
        busy.status->flags=1;
        completed.tick(r,busy,true,busy.status_received_at,30);
        check(completed.phase==P::fired);
    }
    c2::fcs::Snapshot f;f.connected=true;f.status=c2::fcs::Status{1,0,0};f.status_received_at=later;
    M m;m.target=7;m.generation=1;m.paused_at=start;m.enter(P::fired,start);
    m.tick(r,f,false,later,3600);check(m.phase==P::fired&&m.completed_step()==6);
    auto stale=m;f.status_received_at=start;stale.tick(r,f,false,later,3600);
    check(stale.reason==R::status_pending&&stale.completed_step()==6);f.status_received_at=later;
    auto lost=m;r.connected=false;lost.tick(r,f,false,later,3600);
    check(lost.reason==R::lidar_disconnected&&lost.completed_step()==6);r.connected=true;
    auto output=m;f.status->flags=5;output.tick(r,f,false,later,3600);
    check(output.reason==R::fcs_off_unconfirmed);f.status->flags=1;
    auto pending=m;pending.enter(P::paused,start);pending.tick(r,f,true,later,3600);
    check(pending.phase==P::paused&&pending.reason==R::success&&pending.completed_step()==4);
    auto paused_stale=pending;f.status_received_at=start;
    paused_stale.tick(r,f,true,later,3600);check(paused_stale.reason==R::status_pending);
    f.status_received_at=later;
    auto paused_lost=pending;r.connected=false;
    paused_lost.tick(r,f,true,later,3600);check(paused_lost.reason==R::lidar_disconnected);
    r.connected=true;
    m.enter(P::resuming,later);r.lidar_inhibited=false;r.lidar_on=true;r.lidar_request=1;
    r.lidar_known=false;r.lidar_on=false; // SDK success does not imply emission confirmation.
    check(!M::stopped(r));
    m.tick(r,f,false,later,3600);check(m.phase==P::observing);
    f.status_received_at=later+std::chrono::seconds(3);
    m.tick(r,f,false,f.status_received_at,3603);
    check(m.reason==R::result_unconfirmed&&m.completed_step()==7);
    // Confirmed completion does not make absence evidence of HIT.
    m.capture_reference=c2::rdr::BasePoint{};m.output_seen=true;
    r.lidar_known=r.lidar_on=true;m.enter(P::observing,later);
    auto replacement=m, disconnected=m, no_capture=m;
    m.tick(r,f,false,f.status_received_at,3603);
    check(m.phase==P::failed&&m.state()==3&&m.reason==R::result_unconfirmed);
    no_capture.output_seen=false;no_capture.tick(r,f,false,f.status_received_at,3603);
    check(no_capture.reason==R::result_unconfirmed);
    r.connected=false;disconnected.tick(r,f,false,f.status_received_at,3603);
    check(disconnected.reason==R::lidar_disconnected);r.connected=true;
    c2::rdr::Target other;other.id=99;other.seen=3601;other.track_state=other.detected=1;
    other.status=0;other.pan=1800;other.tilt=0;
    r.targets[99]=other;replacement.tick(r,f,false,f.status_received_at,3603);
    check(replacement.reason==R::result_unconfirmed);
    r.targets[99].seen=3603.1;f.status_received_at=later+std::chrono::milliseconds(3100);
    replacement.tick(r,f,false,f.status_received_at,3603.1);
    check(replacement.phase==P::alive);r.targets.clear();
    { // 2026-11-17: first new ID arrives 245 ms after the old HIT deadline.
        M late;late.target=5;late.generation=1;late.resume_sample=100;late.result_started=true;
        late.output_seen=true;late.capture_reference=c2::rdr::BasePoint{};late.enter(P::observing,start);
        auto step=[&](M& s,double elapsed) {
            f.status_received_at=start+std::chrono::duration_cast<M::Clock::duration>(std::chrono::duration<double>(elapsed));
            s.tick(r,f,false,f.status_received_at,100+elapsed);
        };
        step(late,3);check(late.reason==R::result_unconfirmed&&late.completed_step()==7);
        auto no_data=late,expired=late,session_changed=late,ambiguous=late,predicted=late;
        auto excluded=late,far=late,stale_result=late,same_id=late,device_fault=late;
        other.id=6;other.x=.01;other.seen=103.245;r.targets[6]=other;
        step(late,3.245);check(late.reason==R::result_unconfirmed);
        step(late,3.4);check(late.reason==R::result_unconfirmed); // same packet
        r.targets[6].seen=104.4;step(late,4.4);check(late.phase==P::alive&&late.state()==111);
        r.targets[6].track_state=2;r.targets[6].detected=0;
        step(predicted,4.4);check(predicted.reason==R::result_unconfirmed);
        r.targets[6]=other;r.targets[7]=other;r.targets[7].id=7;
        step(ambiguous,3.245);r.targets[6].seen=r.targets[7].seen=104.4;
        step(ambiguous,4.4);check(ambiguous.reason==R::result_unconfirmed);
        r.targets.erase(7);r.excluded.insert(6);
        step(excluded,4.4);check(excluded.reason==R::result_unconfirmed);r.excluded.clear();
        r.targets[6].x=.2;step(far,4.4);r.targets[6].seen=104.5;step(far,4.5);
        check(far.reason==R::result_unconfirmed);
        r.targets[6]=other;step(stale_result,7);check(stale_result.reason==R::result_unconfirmed);
        r.targets.clear();other.id=5;other.seen=104.5;r.targets[5]=other;
        step(same_id,4.5);check(same_id.phase==P::alive);
        f.status->flags=17;step(device_fault,4.5);check(device_fault.reason==R::device_fault);f.status->flags=1;
        r.targets.clear();other.id=6;r.targets[6]=other;
        r.targets.erase(7);r.targets[6].seen=113.1;step(expired,13.1);
        check(expired.reason==R::result_unconfirmed);
        ++r.generation;step(session_changed,4.4);check(session_changed.reason==R::lidar_disconnected);--r.generation;
        r.targets.clear();step(no_data,13);check(no_data.reason==R::result_unconfirmed);
        step(no_data,60);check(no_data.reason==R::result_unconfirmed&&no_data.state()!=108);
        r.targets.clear();
        auto stalled=no_data;stalled.enter(P::observing,start);step(stalled,60);
        check(stalled.reason==R::result_unconfirmed&&stalled.state()==3);
    }
    M wrapped;wrapped.generation=1;wrapped.pan=.1;wrapped.tilt=0;
    wrapped.enter(P::aiming,later);f.status->azimuth=359.9;
    f.status_received_at=later+std::chrono::milliseconds(100);
    wrapped.tick(r,f,true,f.status_received_at,3600.1);
    check(wrapped.phase==P::aimed);

    // Thermal warnings only permit a completed matching STOP, never an inferred OFF.
    r.connected=r.reporting=r.lidar_supported=r.lidar_inhibited=true;
    r.mode=2;r.lidar_request=0;r.lidar_result=0;r.lidar_known=r.lidar_on=false;
    r.sensor_health_valid=true;
    for(auto thermal:{7,8}){
        r.sensor_state=thermal;check(M::stopped(r));check(!r.lidar_known);
        r.lidar_result=1;check(!M::stopped(r));
        r.lidar_result=5;check(!M::stopped(r));r.lidar_result=0;
        r.sensor_health_valid=false;check(!M::stopped(r));r.sensor_health_valid=true;
        r.lidar_on=true;check(!M::stopped(r));r.lidar_known=true;check(!M::stopped(r));r.lidar_known=r.lidar_on=false;
    }
    r.sensor_state=9;check(!M::stopped(r));
    M hot;hot.generation=1;hot.enter(P::aimed,later);hot.tick(r,f,true,later,3600);
    check(hot.reason==R::lidar_thermal_fault);
    r.sensor_state=7;r.thermal_protective_stop=true;check(!M::stopped(r));
    r.thermal_protective_stop=false;r.lidar_result=5;
    M warn;warn.generation=1;warn.paused_at=later;warn.enter(P::pausing,later);warn.tick(r,f,true,later,3600);
    check(warn.reason==R::lidar_stop_warning_unconfirmed);

    // Health packets have their own request matching, sequence and expiry; never alter optical flags.
    c2::rdr::Session session{42};session.enable_sensor_health();session.connect(0);
    auto hello=session.take().at(0);auto reply=hello;

    c2::rdr::word(reply,0,0xDE01);c2::rdr::word(reply,2,10);
    c2::rdr::word(reply,4,c2::rdr::word(hello,2));c2::rdr::word(reply,20,0x024f);
    session.receive(reply,0);session.tick(.1);auto outgoing=session.take();
    auto poll=*std::find_if(outgoing.begin(),outgoing.end(),[](auto&p){return c2::rdr::word(p,0)==0xED08;});
    reply=poll;c2::rdr::word(reply,0,0xDE08);c2::rdr::word(reply,2,11);
    c2::rdr::word(reply,4,c2::rdr::word(poll,2));c2::rdr::word(reply,6,7);c2::rdr::word(reply,8,1);
    auto bad=reply;c2::rdr::word(bad,12,999);session.receive(bad,.2);check(!session.state().sensor_health_valid);
    bad=reply;c2::rdr::word(bad,8,4);session.receive(bad,.2);check(!session.state().sensor_health_valid);
    session.receive(reply,.2);check(session.state().thermal_warning()&&!session.state().lidar_known);
    session.tick(1.2);outgoing=session.take();
    poll=*std::find_if(outgoing.begin(),outgoing.end(),[](auto&p){return c2::rdr::word(p,0)==0xED08;});
    c2::rdr::word(reply,4,c2::rdr::word(poll,2));c2::rdr::word(reply,6,9);
    session.receive(reply,1.3);check(session.state().sensor_state==7); // same sequence is stale
    c2::rdr::word(reply,2,12);c2::rdr::word(reply,8,3);session.receive(reply,1.3);
    check(session.state().thermal_fault());
    session.tick(2.3);outgoing=session.take();
    poll=*std::find_if(outgoing.begin(),outgoing.end(),[](auto&p){return c2::rdr::word(p,0)==0xED08;});
    c2::rdr::word(reply,4,c2::rdr::word(poll,2));c2::rdr::word(reply,2,13);
    c2::rdr::word(reply,6,3);c2::rdr::word(reply,8,1);session.receive(reply,2.31);
    check(session.state().thermal_fault()); // A later health reply cannot undo a protective latch.
    session.tick(3.81);check(!session.state().sensor_health_valid&&!session.state().connected);
    return 0;
}
