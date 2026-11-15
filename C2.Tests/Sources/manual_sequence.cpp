#include "manual_sequence.hpp"
#include <stdexcept>
int manual_sequence_entry() {
    using M=ManualSequence;using P=M::Phase;using R=M::Result;
    auto check=[](bool ok){if(!ok)throw std::runtime_error("manual wait/progress regression");};
    const auto start=M::Clock::now(),later=start+std::chrono::hours(1);
    c2::rdr::State r;r.connected=r.reporting=r.lidar_supported=r.lidar_known=r.lidar_inhibited=true;
    r.mode=2;r.generation=1;
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
    check(pending.reason==R::pause_timeout&&pending.completed_step()==4);
    m.enter(P::resuming,later);r.lidar_inhibited=false;r.lidar_on=true;r.lidar_request=1;
    r.lidar_known=false;r.lidar_on=false; // SDK success does not imply emission confirmation.
    check(!M::stopped(r));
    m.tick(r,f,false,later,3600);check(m.phase==P::observing);
    f.status_received_at=later+std::chrono::seconds(3);
    m.tick(r,f,false,f.status_received_at,3603);
    check(m.reason==R::result_unconfirmed&&m.completed_step()==7);
    M wrapped;wrapped.generation=1;wrapped.pan=.1;wrapped.tilt=0;
    wrapped.enter(P::aiming,later);f.status->azimuth=359.9;
    f.status_received_at=later+std::chrono::milliseconds(100);
    wrapped.tick(r,f,true,f.status_received_at,3600.1);
    check(wrapped.phase==P::aimed);
    return 0;
}
