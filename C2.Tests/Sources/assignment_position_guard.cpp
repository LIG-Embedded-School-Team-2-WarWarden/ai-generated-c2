#include "assignment_position_guard.hpp"
#include "assignment_health.hpp"
#include <iostream>
#include <stdexcept>

namespace {
void check(bool ok,const char* message) { if (!ok) throw std::runtime_error(message); }
}
int assignment_position_guard_entry() {
    using c2::rdr::BasePoint;
    const BasePoint anchor{2,3,1};
    AssignmentPositionGuard guard({.025,.018,.005});
    check(!guard.consistent(anchor,1),"unassigned position has no reference");
    guard.start(anchor,1);
    AssignmentHealth health;health.start();
    for (int i=1;i<=100;++i) {
        const double sign=i%2?1.:-1.;
        const bool same=guard.consistent({2+sign*.04,3-sign*.03,1+sign*.02},1+i*.1);
        health.update(true,true,true,same,true,false,false,false);
        check(health.state==2,"centimetre jitter must retain completed assignment");
    }
    guard.start(anchor,20);
    check(guard.consistent({2.07,3,1},20.1),"X tolerance follows measurement noise, not old 1cm gate");
    check(guard.consistent({2,3.052,1.049},20.2),"independent Y noise and minimum Z tolerance");
    check(guard.consistent({2,3,1.08},20.3),"one outlying observation is tolerated");
    for (int i=0;i<100;++i)
        check(guard.consistent({2,3,1.08},20.3),"polling cannot confirm displacement");
    check(guard.consistent(anchor,20.4),"return to reference clears isolated excursion");
    check(guard.consistent({2.2,3,1},20.5),"first displaced sample remains pending");
    check(guard.consistent({2.2,3,1},20.6),"second displaced sample remains pending");
    const bool same=guard.consistent({2.2,3,1},20.7);
    check(!same,"three fresh displaced samples require reassignment");
    health.update(true,true,true,same,true,false,false,false);
    check(health.state==3 && health.reason==c2::ipc::Result::target_changed,"confirmed movement still invalidates alignment");
    guard.start(anchor,30);
    for (int i=1;i<=6;++i) guard.consistent({2+i*.02,3,1},30+i*.1);
    check(!guard.consistent({2.14,3,1},30.7),"slow movement is compared to assignment reference, not previous sample");
    guard.reset();guard.start({4,5,2},40);
    check(guard.consistent({4.02,5,2},40.1),"reassignment clears old reference and confirmation count");
    health.reset();health.start();health.update(true,false,true,true,true,false,false,false);
    check(health.reason==c2::ipc::Result::device_fault,"device fault handling remains active");
    health.reset();health.start();health.update(true,true,false,false,false,false,false,false);
    check(health.state==4 && health.reason==c2::ipc::Result::target_pending,"missing target remains unavailable");
    std::cout<<"PASS: assignment jitter tolerance, independent axes, fresh-sample confirmation, cumulative movement and lifecycle\n";
    return 0;
}
