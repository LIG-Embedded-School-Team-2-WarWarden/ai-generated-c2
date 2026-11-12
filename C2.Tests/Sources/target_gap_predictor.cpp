#include "target_gap_predictor.hpp"
#include "assignment_position_guard.hpp"
#include <stdexcept>
#include <iostream>

int target_gap_predictor_entry() {
    using namespace c2::rdr;
    auto check=[](bool ok){if(!ok)throw std::runtime_error("gap predictor regression");};
    State state;state.connected=state.reporting=true;state.mode=2;state.generation=1;
    auto& raw=state.targets[7];raw.id=7;raw.status=0;raw.track_state=raw.detected=1;
    raw.pan=1800;raw.tilt=0;raw.z=10;
    TargetGapPredictor predictor;
    auto update=[&](double now) {return predictor.update(state,correct_targets(state,{}),{},now).at(0);};
    raw.seen=10;raw.x=0;check(update(10).valid);
    check(!update(10.3).valid); // One observation cannot establish motion.
    raw.seen=10.1;raw.x=-.1;check(update(10.1).valid);
    check(std::abs(update(10.2).world.east-.1)<1e-9);
    check(std::abs(update(10.5).world.east-.5)<1e-9);
    check(std::abs(update(10.5).world.east-.5)<1e-9); // No recursive extrapolation.
    check(raw.x==-.1 && raw.seen==10.1 && raw.track_state==1);
    check(update(11.59).valid);check(!update(11.61).valid);
    raw.seen=11.7;raw.x=-2;check(update(11.7).valid);check(!update(12).valid);
    raw.seen=11.8;raw.x=-2.1;update(11.8);check(update(12.1).valid);
    raw.track_state=2;raw.detected=0;raw.seen=12.2;raw.x=-3;
    check(update(12.2).world.east==3);check(!update(12.5).valid);
    raw.track_state=0;raw.status=2;check(!update(12.6).valid);
    raw.track_state=raw.detected=1;raw.status=0;raw.seen=13;update(13);
    check(!update(13.3).valid); // LOST erased velocity.
    raw.seen=13.1;raw.x=-3.1;update(13.1);check(update(13.4).valid);
    ++state.generation;update(13.4);check(!update(13.4).valid);
    raw.seen=13.2;raw.x=-3.2;update(13.2);
    state.excluded.insert(7);check(!update(13.5).valid);state.excluded.clear();
    update(13.2);check(!update(13.5).valid);
    raw.seen=13.3;raw.x=-3.3;update(13.3);
    const auto shifted=predictor.update(state,correct_targets(state,{1,0,0}),{1,0,0},13.6);
    check(!shifted[0].valid);
    state.connected=false;update(13.3);state.connected=true;check(!update(13.6).valid);
    AssignmentPositionGuard guard({.05,.05,.05});guard.start({0,0,0},1);
    check(guard.prediction_consistent({.1,0,0}));check(!guard.prediction_consistent({.2,0,0}));
    check(guard.outside_samples()==0);
    std::cout<<"PASS: camera gap prediction, expiry, sender precedence, lifecycle and sample isolation\n";
    return 0;
}
