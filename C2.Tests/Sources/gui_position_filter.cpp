#include "target_position_filter.hpp"
#include <iostream>
#include <stdexcept>

namespace {
void check(bool value, const char* message) { if (!value) throw std::runtime_error(message); }
c2::rdr::State scene() {
    c2::rdr::State state;state.connected=state.reporting=true;state.mode=2;state.generation=1;
    return state;
}
void sample(c2::rdr::State& state, double time, c2::rdr::BasePoint world, uint16_t id=7) {
    auto& raw=state.targets[id];raw.id=id;raw.status=0;raw.track_state=raw.detected=1;
    raw.pan=1800;raw.tilt=0;raw.seen=time;raw.x=-world.east;raw.y=world.up;raw.z=world.north;
}
c2::rdr::BasePoint point(c2::rdr::TargetPositionFilter& filter, const c2::rdr::State& state,
                         c2::rdr::BasePoint origin={}) {
    return filter.update(state,origin).at(0).world;
}
bool equal(c2::rdr::BasePoint a,c2::rdr::BasePoint b) {
    return a.east==b.east && a.north==b.north && a.up==b.up;
}
}
int gui_position_filter_entry() {
    using namespace c2::rdr;
    auto state=scene();TargetPositionFilter filter,once;
    sample(state,1,{2,3,1});
    check(equal(point(filter,state),{2,3,1}),"first sample is displayed without startup drift");
    point(once,state);
    sample(state,1.1,{2.05,2.95,1.05});
    const auto smoothed=point(filter,state);
    point(once,state);
    check(smoothed.east>2 && smoothed.east<2.05 && smoothed.north>2.95 && smoothed.north<3 &&
        smoothed.up>1 && smoothed.up<1.05,"all three axes smooth a new measurement");
    for(int i=0;i<100;++i) check(equal(point(filter,state),smoothed),"GUI redraws do not reprocess one sample");
    sample(state,1.2,{1.95,3.05,.95});
    check(equal(point(filter,state),point(once,state)),"query frequency cannot change filter confidence");
    check(state.targets[7].x==-1.95 && state.targets[7].seen==1.2,"raw measurement remains unchanged");

    TargetPositionFilter noisy;auto noise_state=scene();double raw_squared=0,filtered_squared=0;
    uint32_t seed=12345;
    auto noise=[&] {seed=1664525u*seed+1013904223u;return (double(seed)/4294967296.-.5)*.1732050807568877;};
    for(int i=0;i<1000;++i) {
        BasePoint measured{2+noise(),3+noise(),1+noise()};sample(noise_state,i*.1,measured);
        const auto output=point(noisy,noise_state);
        if(i>=100) {
            raw_squared+=std::pow(measured.east-2,2)+std::pow(measured.north-3,2)+std::pow(measured.up-1,2);
            filtered_squared+=std::pow(output.east-2,2)+std::pow(output.north-3,2)+std::pow(output.up-1,2);
        }
    }
    const double rms_ratio=std::sqrt(filtered_squared/raw_squared);
    check(rms_ratio<.65,"stationary synthetic noise RMS must drop materially");
    const auto before=point(noisy,noise_state);
    sample(noise_state,100,{3,4,2});const auto first_step=point(noisy,noise_state);
    check(first_step.east>before.east && first_step.east<3,"movement is smoothed, not frozen");
    for(int i=1;i<=20;++i) {sample(noise_state,100+i*.1,{3,4,2});point(noisy,noise_state);}
    check(std::abs(point(noisy,noise_state).east-3)<.0001,"filter follows a changed position");

    auto resets=scene();TargetPositionFilter reset_filter;
    sample(resets,1,{1,2,3});point(reset_filter,resets);
    resets.targets[7].status=2;resets.targets[7].track_state=0;
    check(!reset_filter.update(resets,{})[0].valid,"lost sample is invalid");
    sample(resets,2,{8,9,10});check(equal(point(reset_filter,resets),{8,9,10}),"reacquisition starts fresh");
    sample(resets,2.1,{20,30,40});resets.generation++;
    check(equal(point(reset_filter,resets),{20,30,40}),"new connection session resets estimates");
    check(equal(point(reset_filter,resets,{1,2,3}),{21,32,43}),"installation edits reset estimates immediately");
    sample(resets,7,{4,5,6});check(equal(point(reset_filter,resets,{1,2,3}),{5,7,9}),"long receive gaps reset estimates");
    sample(resets,1,{9,8,7});check(equal(point(reset_filter,resets,{1,2,3}),{10,10,10}),"backward receive time resets estimates");
    resets.targets[7].pan=-32768;reset_filter.update(resets,{});
    sample(resets,2,{3,4,5});check(equal(point(reset_filter,resets),{3,4,5}),"unknown pose clears history");
    resets.targets.clear();reset_filter.update(resets,{});
    sample(resets,3,{10,11,12});check(equal(point(reset_filter,resets),{10,11,12}),"removed ID clears history");
    resets.connected=false;reset_filter.update(resets,{});resets.connected=true;
    sample(resets,4,{5,6,7});check(equal(point(reset_filter,resets),{5,6,7}),"disconnect clears history");
    resets.reporting=false;reset_filter.update(resets,{});resets.reporting=true;
    sample(resets,4.1,{15,16,17});check(equal(point(reset_filter,resets),{15,16,17}),"reporting off clears history");
    resets.mode=0;reset_filter.update(resets,{});resets.mode=2;
    sample(resets,4.2,{25,26,27});check(equal(point(reset_filter,resets),{25,26,27}),"stop clears history");
    sample(resets,4.3,{25.1,26.1,27.1});point(reset_filter,resets);
    sample(resets,4.3,{100,200,300},19);
    const auto independent=reset_filter.update(resets,{});
    check(independent.size()==2 && equal(independent[1].world,{100,200,300}),"IDs have independent estimates");

    TargetPositionFilter predicted;auto predicted_state=scene();
    sample(predicted_state,1,{1,2,3});point(predicted,predicted_state);
    sample(predicted_state,1.1,{1.1,2.1,3.1});
    const auto before_prediction=point(predicted,predicted_state);
    sample(predicted_state,1.2,{10,20,30});predicted_state.targets[7].track_state=2;predicted_state.targets[7].detected=0;
    const auto prediction_display=point(predicted,predicted_state);
    check(prediction_display.east>before_prediction.east && prediction_display.east<10 &&
        prediction_display.north>before_prediction.north && prediction_display.north<20 &&
        prediction_display.up>before_prediction.up && prediction_display.up<30,
        "sender prediction is smoothed on all axes instead of bypassing the filter");
    for(int i=0;i<100;++i)
        check(equal(point(predicted,predicted_state),prediction_display),"repeated predicted snapshot is not smoothed again");
    check(predicted_state.targets[7].x==-10 && predicted_state.targets[7].track_state==2 &&
        predicted_state.targets[7].detected==0,"raw predicted data and status remain unchanged");
    sample(predicted_state,1.3,{10,20,30});
    const auto observed_again=point(predicted,predicted_state);
    check(observed_again.east>prediction_display.east && observed_again.east<10,
        "observed update resumes smoothly from the predicted display state");
    predicted_state.targets[7].track_state=0;predicted_state.targets[7].status=2;
    check(!predicted.update(predicted_state,{})[0].valid,"loss clears the smoothed predicted state");

    for(double process_stddev:{.1,.03}) {
        TargetPositionFilter with_predictions({true,.05,process_stddev}),without_predictions({true,.05,process_stddev});
        auto covariance_state=scene();sample(covariance_state,1,{1,2,3});
        point(with_predictions,covariance_state);point(without_predictions,covariance_state);
        sample(covariance_state,1.1,{1.1,2.1,3.1});
        const auto stationary=point(with_predictions,covariance_state);point(without_predictions,covariance_state);
        // Equal predicted positions leave the mean unchanged, exposing any
        // false covariance shrinkage through the next real update's gain.
        for(int i=1;i<=10;++i) {
            sample(covariance_state,1.1+i*.1,stationary);
            covariance_state.targets[7].track_state=2;covariance_state.targets[7].detected=0;
            point(with_predictions,covariance_state);
        }
        sample(covariance_state,2.2,{4,5,6});
        const auto with=point(with_predictions,covariance_state),without=point(without_predictions,covariance_state);
        check(std::abs(with.east-without.east)<1e-12 && std::abs(with.north-without.north)<1e-12 &&
            std::abs(with.up-without.up)<1e-12,"predictions do not increase measurement confidence");
    }
    TargetPositionFilter predicted_first;auto initial_prediction=scene();
    sample(initial_prediction,1,{2,3,4});initial_prediction.targets[7].track_state=2;initial_prediction.targets[7].detected=0;
    check(equal(point(predicted_first,initial_prediction),{2,3,4}),"first predicted sample initializes the display");
    sample(initial_prediction,1.1,{3,4,5});initial_prediction.targets[7].track_state=2;initial_prediction.targets[7].detected=0;
    const auto next_prediction=point(predicted_first,initial_prediction);
    check(next_prediction.east>2 && next_prediction.east<3,"predicted-only stream is smoothed after initialization");

    TargetPositionFilter rotated;auto rotated_state=scene();sample(rotated_state,1,{2,3,1});point(rotated,rotated_state);
    auto& rotated_raw=rotated_state.targets[7];rotated_raw.x=3;rotated_raw.y=1;rotated_raw.z=2;rotated_raw.pan=2700;rotated_raw.seen=1.1;
    const auto invariant=point(rotated,rotated_state);
    check(std::abs(invariant.east-2)<1e-12 && std::abs(invariant.north-3)<1e-12 && invariant.up==1,
        "filter operates in world axes after observation pose correction");

    auto disabled_config=parse_target_position_filter({{"enabled",false}});TargetPositionFilter disabled(disabled_config);
    sample(state,1,{1,2,3});point(disabled,state);sample(state,1.1,{9,8,7});
    check(equal(point(disabled,state),{9,8,7}),"disabled filter preserves live coordinates");
    sample(state,1.2,{11,12,13});state.targets[7].track_state=2;state.targets[7].detected=0;
    check(equal(point(disabled,state),{11,12,13}),"disabled filter also preserves predicted coordinates");

    auto stable_config=parse_target_position_filter({{"measurement_stddev_m",.03},
        {"process_stddev_m_sqrt_s",.01},{"adaptive_motion",true},
        {"motion_process_stddev_m_sqrt_s",.08},{"motion_trigger_sigma",2.5}});
    auto fixed_config=stable_config;fixed_config.adaptive_motion=false;
    TargetPositionFilter adaptive(stable_config),fixed(fixed_config);auto moving=scene();
    for(int i=0;i<50;++i) {sample(moving,i,{0,0,0});point(adaptive,moving);point(fixed,moving);}
    sample(moving,50,{.2,0,0});
    const auto first_move=point(adaptive,moving);
    check(equal(first_move,point(fixed,moving)),"one unusual observation alone does not boost motion gain");
    for(int i=0;i<100;++i)check(equal(point(adaptive,moving),first_move),"redraws cannot confirm movement");
    sample(moving,51,{.2,0,0});
    check(point(adaptive,moving).east>=.18 && point(fixed,moving).east<.18,
        "two consistent observed changes reach 90 percent of a 20 cm step");
    sample(moving,52,{.2,0,0});moving.targets[7].status=2;moving.targets[7].track_state=0;
    adaptive.update(moving,{});fixed.update(moving,{});
    sample(moving,53,{0,0,0});point(adaptive,moving);point(fixed,moving);
    sample(moving,54,{.2,0,0});
    check(equal(point(adaptive,moving),point(fixed,moving)),"loss clears motion confirmation and boost history");

    for(bool predicted_spike:{false,true}) {
        TargetPositionFilter robust(stable_config),reference(fixed_config);auto spikes=scene();
        for(int i=0;i<50;++i) {sample(spikes,i,{0,0,0});point(robust,spikes);point(reference,spikes);}
        sample(spikes,50,{.2,0,0});point(robust,spikes);point(reference,spikes);
        sample(spikes,51,predicted_spike?BasePoint{.3,0,0}:BasePoint{-.2,0,0});
        if(predicted_spike) {spikes.targets[7].track_state=2;spikes.targets[7].detected=0;}
        check(equal(point(robust,spikes),point(reference,spikes)),
            "predictions and direction-reversing spikes do not confirm movement");
        if(predicted_spike) {
            // Fresh predictions must not keep an old observed candidate alive.
            for(int i=52;i<=54;++i) {spikes.targets[7].seen=i;point(robust,spikes);point(reference,spikes);}
            sample(spikes,55,{.5,0,0});
            check(equal(point(robust,spikes),point(reference,spikes)),"stale observed motion candidate expires");
        }
    }
    TargetPositionFilter small_move(stable_config);auto small=scene();
    for(int i=0;i<50;++i) {sample(small,i,{0,0,0});point(small_move,small);}
    for(int i=50;i<58;++i) {sample(small,i,{.05,0,0});point(small_move,small);}
    check(point(small_move,small).east>=.045,"small changes below the motion threshold are not frozen");
    bool bad_motion=false;
    try {auto bad=stable_config;bad.motion_process_stddev_m_sqrt_s=.001;TargetPositionFilter reject(bad);}
    catch(const std::invalid_argument&) {bad_motion=true;}
    check(bad_motion,"adaptive process noise cannot be lower than stationary noise");
    auto axes_config=parse_target_position_filter({{"measurement_stddev_xyz_m",{.06,.03,.015}},
        {"process_stddev_xyz_m_sqrt_s",{.01,.01,.01}}});
    TargetPositionFilter axes(axes_config);auto axis_state=scene();
    for(int i=0;i<50;++i){sample(axis_state,i,{0,0,0});point(axes,axis_state);}
    sample(axis_state,50,{.1,.1,.1});const auto different_gains=point(axes,axis_state);
    check(different_gains.east<different_gains.north && different_gains.north<different_gains.up,
        "noisier world axes trust identical new observations less");
    auto equal_axes=stable_config;
    equal_axes.measurement_stddev_xyz_m=std::array<double,3>{.03,.03,.03};
    equal_axes.process_stddev_xyz_m_sqrt_s=std::array<double,3>{.01,.01,.01};
    equal_axes.motion_process_stddev_xyz_m_sqrt_s=std::array<double,3>{.08,.08,.08};
    TargetPositionFilter scalar(stable_config),explicit_axes(equal_axes);auto compatibility=scene();
    for(int i=0;i<100;++i) {
        sample(compatibility,i*.1,{i<50?0.:.3,i<70?0.:.2,.001*(i%3)});
        if(i%7==0){compatibility.targets[7].track_state=2;compatibility.targets[7].detected=0;}
        check(equal(point(scalar,compatibility),point(explicit_axes,compatibility)),
            "explicit equal XYZ settings preserve scalar behavior including motion and predictions");
    }
    TargetPositionFilter axes_predictions(axes_config),axes_gap(axes_config);auto confidence=scene();
    sample(confidence,0,{0,0,0});point(axes_predictions,confidence);point(axes_gap,confidence);
    for(int i=1;i<=10;++i) {
        sample(confidence,i*.1,{0,0,0});confidence.targets[7].track_state=2;confidence.targets[7].detected=0;
        point(axes_predictions,confidence);
    }
    sample(confidence,1.1,{.1,.1,.1});
    const auto with_pred=point(axes_predictions,confidence),without_pred=point(axes_gap,confidence);
    check(std::abs(with_pred.east-without_pred.east)<1e-12 &&
        std::abs(with_pred.north-without_pred.north)<1e-12 && std::abs(with_pred.up-without_pred.up)<1e-12,
        "predictions do not shrink covariance on any independently tuned axis");
    for(const auto& bad_axes:{nlohmann::json::array({.01,.02}),nlohmann::json::array({.01,0,.02}),
        nlohmann::json::array({.01,"bad",.02}),nlohmann::json::array({.01,-.01,.02})}) {
        bool rejected=false;
        try{TargetPositionFilter reject(parse_target_position_filter({{"measurement_stddev_xyz_m",bad_axes}}));}
        catch(const std::exception&){rejected=true;}
        check(rejected,"invalid XYZ noise arrays are rejected before device startup");
    }
    for(double invalid:{0.,-1.,std::numeric_limits<double>::infinity(),1e-300,1e300}) {
        bool rejected=false;
        try { TargetPositionFilter bad({true,invalid,.1}); }catch(const std::invalid_argument&){rejected=true;}
        check(rejected,"invalid noise parameters are rejected before starting devices");
    }
    std::cout<<"PASS: GUI Kalman lifecycle, independent IDs, sample deduplication, prediction handling; synthetic RMS ratio="<<rms_ratio<<'\n';
    return 0;
}
