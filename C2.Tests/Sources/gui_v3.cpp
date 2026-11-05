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
    gui::Frame frame;
    frame.payload.rdr_count=frame.payload.fcs_count=1;
    frame.payload.rdr.azimuth=150.3f;frame.payload.rdr.elevation=12.3f;
    frame.payload.rdr.position_z=3.25f;frame.payload.fcs.position_z=-.5f;
    frame.payload.target_count=1;frame.payload.targets[0].azimuth=123.4f;
    auto bytes=gui::serialize(frame);
    check(bytes.size()==492 && bytes[4]==3 && bytes[8]==0xec && bytes[9]==1,"GUI v3 header bytes");
    check(wire_float(bytes,48)==150.3f && wire_float(bytes,52)==12.3f,"angles must be IEEE float at GUI offsets");
    check(wire_float(bytes,64)==3.25f && wire_float(bytes,108)==-.5f,"asset Z offsets");
    check(wire_float(bytes,136)==123.4f,"target offset shifted by both added Z fields");
    gui::Frame decoded;
    check(gui::deserialize(bytes,decoded),"v3 frame roundtrip");
    auto old=bytes;old[4]=2;check(!gui::deserialize(old,decoded),"reject old GUI version");
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
    driver.state.mode=2;driver.state.last_result=0;driver.state.pan=-153;driver.state.tilt=123;
    driver.state.targets[7]={7,.469,-.109,2.229,0,0};adapter.tick(2);
    gui::Payload payload;adapter.populate(payload);
    check(payload.rdr.azimuth==150.3f && payload.rdr.elevation==12.3f,"RDR uses 0.1 degree wire units");
    check(payload.rdr.operating && payload.targets[0].valid,"active observations valid");
    command.target_id=0;check(adapter.set_operating(command).result==0,"GUI stop accepted");
    adapter.populate(payload);
    check(!payload.rdr.operating && !payload.targets[0].valid,"invalidate immediately on stop before ACK");
    command.target_id=2;check(adapter.set_operating(command).result==1,"reject invalid operating value");
    check(std::abs(fcs_motor_to_gui_azimuth(180.2)-134.8f)<.001,"preserve fractional FCS angle");
    check(fcs_motor_to_gui_azimuth(0)==315.f,"FCS full circle display");
    GuiDriver cancelled;cancelled.state.connected=true;cancelled.state.generation=1;
    RdrControl cancel_adapter(cancelled,[](auto&){},0);
    command.target_id=1;cancel_adapter.set_operating(command);
    cancel_adapter.cancel_start(); // CLI stop must cancel a pending GUI start chain.
    cancelled.state.reporting=true;cancelled.state.last_result=0;cancel_adapter.tick(1);
    check(cancelled.commands.size()==1,"CLI stop must prevent a delayed GUI start");
    return 0;
}
