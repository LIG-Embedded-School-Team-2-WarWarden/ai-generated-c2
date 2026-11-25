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
    { // The OFF reply alone is insufficient, even when capture was brief.
        M capture;capture.target=7;capture.generation=1;capture.enter(P::stopping_capture,start);
        c2::fcs::Snapshot report;report.connected=true;report.status=c2::fcs::Status{1,0,0};
        report.status_received_at=start;
        capture.tick(r,report,true,start,0);
        check(capture.state()==112&&capture.completed_step()==5);
        report.status->flags=11;
        report.status_received_at=start+std::chrono::seconds(1);
        capture.tick(r,report,true,report.status_received_at,1);
        check(capture.phase==P::stopping_capture);
        auto delayed=capture;
        report.status_received_at=start+M::capture_completion_timeout;
        delayed.tick(r,report,true,report.status_received_at,30);
        check(delayed.phase==P::failed&&delayed.reason==R::fcs_off_unconfirmed&&delayed.completed_step()==5);
        report.status->flags=1;
        capture.tick(r,report,true,report.status_received_at,30);
        check(capture.phase==P::fired&&capture.target==7);
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
    check(m.phase==P::hit&&m.state()==108&&m.reason==R::success&&m.completed_step()==8);

    // A fresh observation of the original ID still confirms ALIVE immediately.
    M same_id;same_id.target=7;same_id.generation=1;same_id.resume_sample=3600;
    same_id.capture_reference=c2::rdr::BasePoint{};same_id.enter(P::observing,later);
    c2::rdr::Target other;other.id=7;other.seen=3601;other.track_state=other.detected=1;
    other.status=0;other.pan=1800;other.tilt=0;r.targets[7]=other;
    f.status_received_at=later+std::chrono::seconds(1);
    same_id.tick(r,f,false,f.status_received_at,3601);
    check(same_id.phase==P::alive&&same_id.state()==111);

    // A changed ID must still provide two nearby observations before the HIT deadline.
    M replacement;replacement.target=7;replacement.generation=1;replacement.resume_sample=3600;
    replacement.capture_reference=c2::rdr::BasePoint{};replacement.enter(P::observing,later);
    r.targets.clear();other.id=99;other.x=.01;other.seen=3601;r.targets[99]=other;
    replacement.tick(r,f,false,f.status_received_at,3601);
    check(replacement.phase==P::observing);
    r.targets[99].seen=3602;f.status_received_at=later+std::chrono::seconds(2);
    replacement.tick(r,f,false,f.status_received_at,3602);
    check(replacement.phase==P::alive&&replacement.state()==111);

    // Device and session failures remain failures; only clean absence becomes HIT.
    M disconnected;disconnected.target=7;disconnected.generation=1;disconnected.resume_sample=3600;
    disconnected.enter(P::observing,later);r.targets.clear();r.connected=false;
    disconnected.tick(r,f,false,f.status_received_at,3602);
    check(disconnected.reason==R::lidar_disconnected);r.connected=true;
    M device_fault;device_fault.target=7;device_fault.generation=1;device_fault.resume_sample=3600;
    device_fault.enter(P::observing,later);f.status->flags=17;
    device_fault.tick(r,f,false,f.status_received_at,3602);
    check(device_fault.reason==R::device_fault);f.status->flags=1;

    M absent;absent.target=7;absent.generation=1;absent.resume_sample=3600;
    absent.capture_reference=c2::rdr::BasePoint{};absent.enter(P::observing,later);
    f.status_received_at=later+std::chrono::seconds(3);
    absent.tick(r,f,false,f.status_received_at,3603);
    check(absent.phase==P::hit&&absent.state()==108&&absent.reason==R::success);
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
