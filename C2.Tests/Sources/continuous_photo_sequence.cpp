#include "continuous_photo_sequence.hpp"
#include <source_location>
#include <stdexcept>
#include <string>

namespace {
using S=ContinuousPhotoSequence;using R=S::Result;
void check(bool ok,const std::source_location loc=std::source_location::current()) {
    if(!ok)throw std::runtime_error("continuous photo regression at line "+std::to_string(loc.line()));
}
struct Fixture {
    S sequence;
    c2::fcs::Snapshot fcs;
    S::Clock::time_point now=S::Clock::now();
    std::vector<BatchSequence::PhotoTarget> targets{{9,180,10,10},{7,200,20,10}};
    std::vector<S::Entry> moves;
    int captures{},offs{};
    R capture_result=R::success,off_result=R::success,point_result=R::success;
    S::Actions actions;
    Fixture() {
        fcs.connected=true;fcs.status=c2::fcs::Status{1,10,180};fcs.status_received_at=now;
        actions.clock=[&]{return now;};
        actions.point=[&](double tilt,double pan){moves.push_back({0,pan,tilt});return point_result;};
        actions.capture=[&]{++captures;return capture_result;};
        actions.off=[&]{++offs;return off_result;};
    }
    S::Context context(bool paused=true) {return {fcs,paused,now};}
    void arrive() {
        const auto& move=moves.back();now+=std::chrono::milliseconds(100);
        fcs.status->azimuth=move.pan;fcs.status->elevation=move.tilt;fcs.status->flags=1;
        fcs.status_received_at=now;sequence.tick(context(),actions);
    }
};
}

int continuous_photo_sequence_entry() {
    {
        const std::vector<BatchSequence::PhotoTarget> targets{{9,180,10,10},{7,200,20,10}};
        const auto plan=S::make_plan(targets,.05,1);
        check(plan.size()==18&&plan[0].target==9&&plan[1].target==7);
        check(plan[2].target==9&&plan[2].pan==180&&plan[2].tilt==10.3);
        check(plan[4].target==9&&plan[4].pan==180.3&&plan[4].tilt==10.3);
        check(plan[6].target==9&&plan[6].pan==180.3&&plan[6].tilt==10);
        check(plan[16].target==9&&plan[16].pan==179.7&&plan[16].tilt==10.3);
    }
    {
        Fixture x;
        check(x.sequence.start(x.targets,.05,1,x.context(),x.actions)==R::success);
        check(x.sequence.total()==18&&x.moves.size()==1&&x.sequence.confirmed()==0);
        x.sequence.tick(x.context(),x.actions);check(x.captures==0);
        while(x.sequence.running())x.arrive();
        check(x.sequence.phase()==S::Phase::completed&&x.sequence.confirmed()==18);
        check(x.captures==18&&x.moves.size()==18&&x.offs==1&&x.sequence.holds_batch());
        x.sequence.tick(x.context(),x.actions);check(x.captures==18&&x.offs==1);
        x.now+=std::chrono::milliseconds(100);x.fcs.status_received_at=x.now;x.fcs.status->flags=1;
        x.sequence.tick(x.context(),x.actions);check(!x.sequence.holds_batch());
    }
    {
        Fixture x;
        // A FE02 racing with EF03 is older than the post-reply floor and must
        // not authorize a capture. Only the next fresh FE02 may do so.
        x.actions.point=[&](double tilt,double pan) {
            x.moves.push_back({0,pan,tilt});
            x.now+=std::chrono::milliseconds(10);
            x.fcs.status->azimuth=pan;x.fcs.status->elevation=tilt;
            x.fcs.status_received_at=x.now;
            return R::success;
        };
        check(x.sequence.start(x.targets,.05,1,x.context(),x.actions)==R::success);
        x.sequence.tick(x.context(),x.actions);check(x.captures==0);
        x.now+=std::chrono::milliseconds(1);x.fcs.status_received_at=x.now;
        x.sequence.tick(x.context(),x.actions);check(x.captures==1);
    }
    {
        Fixture x;
        // Likewise, a clear FE02 received while EF04(0) is in flight cannot
        // release batch ownership or allow a new run/mutation.
        x.actions.off=[&] {
            ++x.offs;x.now+=std::chrono::milliseconds(10);
            x.fcs.status->flags=1;x.fcs.status_received_at=x.now;
            return R::success;
        };
        check(x.sequence.start(x.targets,.05,1,x.context(),x.actions)==R::success);
        check(x.sequence.stop(x.context(),x.actions)==R::success);
        check(x.sequence.holds_batch());
        x.sequence.tick(x.context(),x.actions);check(x.sequence.holds_batch());
        check(x.sequence.start(x.targets,.05,1,x.context(),x.actions)==R::device_busy);
        x.now+=std::chrono::milliseconds(1);x.fcs.status_received_at=x.now;
        x.sequence.tick(x.context(),x.actions);check(!x.sequence.holds_batch());
    }
    {
        Fixture x;check(x.sequence.start(x.targets,.05,1,x.context(),x.actions)==R::success);
        x.capture_result=R::response_timeout;x.arrive();
        check(x.sequence.phase()==S::Phase::failed&&x.sequence.reason()==R::response_timeout);
        check(x.captures==1&&x.sequence.confirmed()==0&&x.offs==1);
        check(x.sequence.stop(x.context(),x.actions)==R::response_timeout);
        x.sequence.tick(x.context(),x.actions);check(x.captures==1);
        // GetStatus maps a failed phase back to this persisted reason.
        check((x.sequence.phase()==S::Phase::failed?x.sequence.reason():R::success)==R::response_timeout);
    }
    {
        Fixture x;check(x.sequence.start(x.targets,.05,1,x.context(),x.actions)==R::success);
        check(x.sequence.stop(x.context(),x.actions)==R::success&&x.sequence.phase()==S::Phase::stopped);
        check(x.sequence.confirmed()==0&&x.offs==1&&x.captures==0);
    }
    {
        Fixture x;check(x.sequence.start(x.targets,.05,1,x.context(),x.actions)==R::success);
        x.now+=std::chrono::seconds(61);x.fcs.status_received_at=x.now;
        x.sequence.tick(x.context(),x.actions);
        check(x.sequence.phase()==S::Phase::failed&&x.sequence.reason()==R::alignment_required&&x.offs==1);
    }
    {
        Fixture x;x.off_result=R::response_timeout;
        check(x.sequence.start(x.targets,.05,1,x.context(),x.actions)==R::success);
        check(x.sequence.stop(x.context(),x.actions)==R::fcs_off_unconfirmed);
        check(x.sequence.phase()==S::Phase::failed&&x.sequence.reason()==R::fcs_off_unconfirmed);
    }
    {
        InstallationConfig settings;check(settings.continuous_photo==std::array<double,3>{.05,1,0});
        settings.continuous_photo={.05,2,0};validate_camera_settings(settings);
        bool rejected=false;try{settings.continuous_photo={.05,1.5,0};validate_camera_settings(settings);}
        catch(const std::invalid_argument&){rejected=true;}check(rejected);
    }
    {
        for(unsigned command=1210;command<=1212;++command) {
            c2::ipc::PipeCommand request;request.command=command;request.request_id=3;
            c2::ipc::PipeCommand decoded;
            check(c2::ipc::deserialize(c2::ipc::serialize(request),decoded)&&decoded.command==command);
            c2::ipc::PipeResponse response;response.command=command;response.request_id=3;
            response.target_id=1610;response.assignment=4;response.active=1609;response.connected=1;
            c2::ipc::PipeResponse parsed;
            check(c2::ipc::deserialize(c2::ipc::serialize(response),parsed)&&parsed.active==1609);
            response.active=1611;check(!c2::ipc::deserialize(c2::ipc::serialize(response),parsed));
        }
    }
    return 0;
}
