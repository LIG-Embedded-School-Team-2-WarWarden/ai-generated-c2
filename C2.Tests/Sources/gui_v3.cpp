#include "protocol.hpp"
#include "rdr_control.hpp"
#include "fcs_angles.hpp"
#include <bit>
#include <cmath>
#include <limits>
#include <stdexcept>

static void check(bool ok,const char* message) { if(!ok)throw std::runtime_error(message); }
static float wire_float(const std::vector<uint8_t>& b,size_t n) {
    return std::bit_cast<float>(uint32_t(b[n])|(uint32_t(b[n+1])<<8)|
        (uint32_t(b[n+2])<<16)|(uint32_t(b[n+3])<<24));
}
struct GuiDriver {
    c2::rdr::State state;
    std::vector<std::pair<uint16_t,uint16_t>> commands;
    void tick() {}
    auto snapshot() {return state;}
    uint16_t connect() {return 0;}
    uint16_t command(uint16_t msg,uint16_t a=0,uint16_t=0) {
        commands.emplace_back(msg,a);state.last_result=1;return 1;
    }
};
int gui_v3_entry() {
    namespace gui=c2::ipc;
    for(unsigned code=15;code<=24;++code) {
        gui::PipeResponse response;response.command=3;response.result=code;
        gui::PipeResponse parsed_response;
        check(gui::deserialize(gui::serialize(response),parsed_response) && parsed_response.result==code,
            "extended diagnostic response roundtrip");
    }
    gui::PipeResponse invalid_response;invalid_response.result=25;
    gui::PipeResponse parsed_response;
    check(!gui::deserialize(gui::serialize(invalid_response),parsed_response),"reject undefined result");
    gui::Frame frame;
    frame.payload.rdr_count=frame.payload.fcs_count=1;
    frame.payload.rdr.azimuth=150.3f;frame.payload.rdr.elevation=12.3f;
    frame.payload.rdr.position_z=3.25f;frame.payload.fcs.position_z=-.5f;
    frame.payload.target_count=1;frame.payload.targets[0].azimuth=123.4f;
    auto bytes=gui::serialize(frame);
    check(bytes.size()==492 && bytes[4]==3,"unchanged GUI v3 frame layout");
    check(wire_float(bytes,136)==123.4f,"unchanged target field offset");
    gui::Frame decoded;
    check(gui::deserialize(bytes,decoded),"GUI v3 frame roundtrip");
    auto old=bytes;old[4]=4;check(!gui::deserialize(old,decoded),"reject incompatible frame version");
    frame.payload.rdr.azimuth=std::numeric_limits<float>::quiet_NaN();
    check(!gui::deserialize(gui::serialize(frame),decoded),"reject nonfinite GUI angle");
    gui::PipeCommand command;command.command=5;command.request_id=23;command.rdr_id=1;
    command.position_x=1.25f;command.position_y=-2.5f;command.position_z=3;
    bytes=gui::serialize(command);
    check(bytes.size()==40 && bytes[4]==3 && bytes[8]==5 && wire_float(bytes,28)==1.25f &&
        wire_float(bytes,32)==-2.5f && wire_float(bytes,36)==3,"GUI position request layout");
    gui::PipeCommand parsed;
    check(gui::deserialize(bytes,parsed) && parsed.request_id==23,"GUI request correlation");
    bytes.resize(28);check(!gui::deserialize(bytes,parsed),"reject old request length");

    GuiDriver driver;driver.state.connected=true;driver.state.generation=1;
    RdrControl adapter(driver,[](auto&){},0);
    command.command=6;command.fcs_id=1;command.target_id=1;
    check(adapter.set_operating(command).result==0 && driver.commands.back().first==0xed03,"GUI start first enables reporting");
    driver.state.reporting=true;driver.state.last_result=0;adapter.tick(1);
    check(driver.commands.back()==std::pair<uint16_t,uint16_t>{0xed06,1},"start only after reporting confirmed");
    driver.state.mode=2;driver.state.last_result=0;driver.state.pan=1800;driver.state.tilt=123;
    auto& observation=driver.state.targets[7];
    observation.id=7;observation.z=2;observation.status=0;observation.pan=2700;
    observation.tilt=0;observation.detected=1;observation.track_state=1;adapter.tick(2);
    gui::Payload payload;adapter.populate(payload);
    check(payload.rdr.azimuth==135 && payload.rdr.elevation==12.3f,"wire PAN=1800 is legacy GUI center=135");
    check(payload.rdr.operating && payload.targets[0].valid,"active observations valid");
    check(std::abs(payload.targets[0].x-2)<1e-5 && std::abs(payload.targets[0].y)<1e-5 &&
        payload.targets[0].azimuth==90,"use target observation pose with east=90 and no map offset");
    observation.track_state=2;observation.detected=0;adapter.populate(payload);
    check(payload.targets[0].id==7 && payload.targets[0].valid,
        "prediction preserves ID and usable coordinate");
    observation.pan=-32768;adapter.populate(payload);
    check(!payload.targets[0].valid && observation.z==2,
        "unknown pose preserves raw data without inventing a world coordinate");
    observation.pan=2700;

    // Independent 0-degree and 90-degree PAN / 30-degree TILT fixtures for
    // the same fixed world point (east,north,up)=(2,3,1).
    struct PoseFixture { double x,y,z; int pan,tilt; };
    const PoseFixture fixed_point[]{{-2,1,3,1800,0},
        {3,-.1339745962,2.2320508076,2700,300}};
    auto fixed_world_point=[&]() {
        const auto& target=payload.targets[0];
        return payload.target_count==1 && target.id==7 && target.valid &&
            std::abs(target.x-2)<1e-5 && std::abs(target.y-3)<1e-5 &&
            std::abs(target.z-1)<1e-5;
    };
    for(const auto& fixture:fixed_point) for(uint16_t track_state:{1,2}) {
        observation.x=fixture.x;observation.y=fixture.y;observation.z=fixture.z;
        observation.pan=fixture.pan;observation.tilt=fixture.tilt;
        observation.track_state=track_state;observation.detected=track_state==1;
        driver.state.pan=2250;driver.state.tilt=-175;adapter.populate(payload);
        check(fixed_world_point(),"changed observation PAN/TILT preserves observed and predicted world point");
        driver.state.pan=900;driver.state.tilt=450;adapter.populate(payload);
        check(fixed_world_point(),"current device PAN/TILT cannot move an earlier observed or predicted point");
        payload.rdr.position_x=1;payload.rdr.position_y=-2;payload.rdr.position_z=.5;
        adapter.populate(payload);
        check(payload.targets[0].valid && std::abs(payload.targets[0].x-3)<1e-5 &&
            std::abs(payload.targets[0].y-1)<1e-5 && std::abs(payload.targets[0].z-1.5)<1e-5,
            "every GUI entry point sends world XYZ including RDR position at both observation poses");
        payload.rdr.position_x=payload.rdr.position_y=payload.rdr.position_z=0;
        auto raw_preserved=[&]() {
            return observation.x==fixture.x && observation.y==fixture.y && observation.z==fixture.z;
        };
        observation.pan=-32768;adapter.populate(payload);
        check(!payload.targets[0].valid && raw_preserved() && observation.tilt==fixture.tilt,
            "unknown PAN alone invalidates GUI point without changing raw XYZ or TILT");
        observation.pan=fixture.pan;observation.tilt=-32768;adapter.populate(payload);
        check(!payload.targets[0].valid && raw_preserved() && observation.pan==fixture.pan,
            "unknown TILT alone invalidates GUI point without changing raw XYZ or PAN");
        observation.tilt=fixture.tilt;
    }
    // Capture and correct a single observation snapshot before the live driver
    // changes. Publication and pointing must use that same corrected point.
    const auto snapshot=driver.state;
    const auto corrected=c2::rdr::correct_targets(snapshot,{1,-2,.5});
    driver.state.targets[7].pan=1800;driver.state.targets[7].tilt=0;
    driver.state.targets[7].x=99;
    adapter.populate(payload,snapshot,corrected);
    const auto& published=payload.targets[0];
    check(published.valid && std::abs(published.x-3)<1e-5 &&
        std::abs(published.y-1)<1e-5 && std::abs(published.z-1.5)<1e-5,
        "GUI publishes corrected snapshot plus installation position exactly once");
    const auto angles=fcs_point_angles(corrected[0].world.east-.5,
        corrected[0].world.north+.5,corrected[0].world.up-.25);
    check(angles.azimuth==239 && angles.elevation==23.2,
        "FCS pointing uses the published corrected point minus FCS installation position");
    const auto overflow=c2::rdr::correct_targets(snapshot,
        {std::numeric_limits<double>::max(),0,0});
    adapter.populate(payload,snapshot,overflow);
    check(!payload.targets[0].valid && payload.targets[0].x==0 &&
        payload.targets[0].y==0 && payload.targets[0].z==0,
        "unrepresentable installation point is invalid for both GUI and solver");
    command.target_id=0;check(adapter.set_operating(command).result==0,"GUI stop accepted");
    adapter.populate(payload);
    check(!payload.rdr.operating && !payload.targets[0].valid,"invalidate immediately on stop before ACK");
    command.target_id=2;check(adapter.set_operating(command).result==1,"reject invalid operating value");
    check(std::abs(fcs_motor_to_gui_azimuth(180.2)-135.2f)<.001,"preserve fractional FCS angle");
    check(fcs_point_angles(1,1,0).azimuth==225 && fcs_point_angles(-1,1,0).azimuth==135 &&
        fcs_point_angles(0,1,0).azimuth==180,"right target increases PAN; left target decreases PAN");
    const auto live_angles=fcs_point_angles(.458599,1.890695,-.225648);
    check(live_angles.azimuth==193.6 && live_angles.elevation==-6.6,
        "reported target right of FCS commands right-positive PAN 193.6");
    check(fcs_motor_to_gui_azimuth(225)==180 && fcs_motor_to_gui_azimuth(135)==90,
        "GUI map heading matches right and left FCS command directions");
    check(fcs_motor_to_gui_azimuth(0)==315.f,"FCS full circle display");
    GuiDriver cancelled;cancelled.state.connected=true;cancelled.state.generation=1;
    RdrControl cancel_adapter(cancelled,[](auto&){},0);
    command.target_id=1;cancel_adapter.set_operating(command);
    cancel_adapter.cancel_start(); // CLI stop must cancel a pending GUI start chain.
    cancelled.state.reporting=true;cancelled.state.last_result=0;cancel_adapter.tick(1);
    check(cancelled.commands.size()==1,"CLI stop must prevent a delayed GUI start");

    c2::rdr::Target sample;sample.status=0;sample.track_state=1;sample.pan=1800;sample.tilt=0;
    sample.x=1;sample.y=2;sample.z=3;
    auto mirrored=c2::rdr::sensor_to_base(sample);
    check(mirrored && mirrored->east==-1 && mirrored->north==3 && mirrored->up==2 && sample.x==1,
        "positive mirrored lidar X becomes world west and raw X stays unchanged");
    sample.x=-1;
    auto base=c2::rdr::sensor_to_base(sample);
    check(base && base->east==1 && base->north==3 && base->up==2,"north identity changes sensor axis order once");
    sample.pan=2700;sample.tilt=300;base=c2::rdr::sensor_to_base(sample);
    check(base && std::abs(base->east-(3*std::sqrt(3.)/2-1))<1e-9 &&
        std::abs(base->north+1)<1e-9 && std::abs(base->up-(std::sqrt(3.)+1.5))<1e-9,
        "combined pan and tilt rotate in the agreed order");
    check(std::abs(std::hypot(base->east,base->north,base->up)-std::sqrt(14.))<1e-9,
        "rotation preserves sensor range");
    sample.pan=-1800;sample.tilt=900;sample.x=sample.y=0;sample.z=2;base=c2::rdr::sensor_to_base(sample);
    check(base && std::abs(base->north)<1e-9 && std::abs(base->up-2)<1e-9,
        "positive tilt lifts forward and equivalent PAN wraps consistently");
    sample.x=sample.y=sample.z=std::numeric_limits<float>::max();
    check(!c2::rdr::sensor_to_base(sample),"unrepresentable GUI distance cannot publish infinity");
    c2::rdr::State fixed;fixed.connected=fixed.reporting=true;fixed.mode=2;fixed.generation=1;
    auto& tracked=fixed.targets[1];tracked.id=1;tracked.status=0;tracked.track_state=1;
    tracked.pan=1800;tracked.tilt=0;tracked.x=-2;tracked.y=1;tracked.z=3;
    const auto first=c2::rdr::correct_targets(fixed,{1,2,.5})[0];
    tracked.x=-4;tracked.y=2;tracked.z=5;
    const auto latest=c2::rdr::correct_targets(fixed,{1,2,.5})[0];
    check(latest.world.east==5 && latest.world.north==7 && latest.world.up==2.5 &&
        latest.world.east!=first.world.east,"same ID uses the latest observation");
    tracked.track_state=0;tracked.status=2;
    check(!c2::rdr::correct_targets(fixed,{1,2,.5})[0].valid,"loss invalidates world coordinate");
    return 0;
}
