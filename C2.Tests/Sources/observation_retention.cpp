#include "observation_retention.hpp"
#include "protocol.hpp"
#include "assignment_health.hpp"
#include <stdexcept>
static void check(bool b){if(!b)throw std::runtime_error("observation retention contract failed");}
int observation_retention_entry(){
    using namespace c2::rdr;
    State r;r.connected=r.reporting=r.lidar_known=r.lidar_supported=r.lidar_on=true;
    r.mode=2;r.generation=1;
    r.targets[7].seen=1;r.targets[7].track_state=1;r.targets[7].detected=1;
    CorrectedTarget p;p.id=7;p.status=0;p.valid=true;p.world=p.relative={1,3,.5};
    ObservationRetention memory;memory.begin(r,{p},{},1);
    auto control=std::vector{p};r.lidar_inhibited=true;r.lidar_on=false;
    auto display=memory.update(r,control,{},2,false);
    check(display.size()==1&&display[0].id==7&&display[0].status==4&&!display[0].valid&&!control[0].valid);
    // Sensor data can expire or explicitly report LOST during the pause.
    r.targets.clear();control.clear();display=memory.update(r,control,{},5,false);
    check(display.size()==1&&display[0].world.east==1);
    r.lidar_inhibited=false;r.lidar_on=true;
    display=memory.update(r,control,{},6,true);check(display[0].status==5);
    r.targets[7].seen=5.9;r.targets[7].track_state=1;r.targets[7].detected=1;
    control={p};display=memory.update(r,control,{},6.1,true);
    check(!control[0].valid&&display[0].status==5); // Pre-resume data cannot restore validity.
    r.targets[7].seen=6.2;r.targets[7].track_state=2;r.targets[7].detected=0;
    control={p};display=memory.update(r,control,{},6.3,true);check(!control[0].valid);
    r.targets[7].seen=6.4;r.targets[7].track_state=1;r.targets[7].detected=1;
    p.world.east=1.1;control={p};display=memory.update(r,control,{},6.5,true);
    check(control[0].valid&&display.size()==1&&display[0].id==7&&display[0].world.east==1.1);
    // Same coordinates with a new ID must remain a distinct object.
    memory.begin(r,{p},{},7);r.targets.clear();control.clear();memory.update(r,control,{},8,true);
    auto fresh=p;fresh.id=8;r.targets[8].seen=8.1;r.targets[8].track_state=1;r.targets[8].detected=1;
    control={fresh};display=memory.update(r,control,{},8.2,true);
    check(display.size()==2&&display[0].id==7&&display[0].status==5&&display[1].id==8);
    control={fresh};display=memory.update(r,control,{},11,true);check(display.size()==1&&display[0].id==8);
    memory.begin(r,{fresh},{},12);control={fresh};display=memory.update(r,control,{},22,false);
    check(display.size()==1&&display[0].status==2&&!display[0].valid&&!control[0].valid); // Ordinary LOST, no ghost display.
    memory.begin(r,{fresh},{},23);++r.generation;control.clear();check(memory.update(r,control,{},24,false).empty());
    memory.begin(r,{fresh},{},25);control.clear();check(memory.update(r,control,{1,0,0},26,false).empty());
    memory.begin(r,{fresh},{},27);r.connected=false;check(memory.update(r,control,{},28,false).empty());
    r.connected=true;memory.begin(r,{fresh},{},29);r.mode=0;check(memory.update(r,control,{},30,false).empty());
    r.mode=2;memory.begin(r,{fresh},{},31);control.clear();
    check(memory.update(r,control,{},41,true).empty()); // Exact pause boundary also expires.
    // Moving the installation clears only display coordinates, not the gate.
    r.lidar_inhibited=true;r.lidar_on=false;memory.begin(r,{fresh},{},42);
    control={fresh};display=memory.update(r,control,{.01,0,0},42.1,false);
    check(display.size()==1&&display[0].status==2&&!display[0].valid&&!control[0].valid);
    control={fresh};display=memory.update(r,control,{.01,0,0},42.2,false);check(!control[0].valid);
    r.lidar_inhibited=false;r.lidar_on=true;
    control={fresh};memory.update(r,control,{.01,0,0},43,true);check(!control[0].valid);
    r.targets[8].seen=43.1;control={fresh};memory.update(r,control,{.01,0,0},43.2,true);check(control[0].valid);
    control={fresh};memory.update(r,control,{.02,0,0},43.3,true);check(control[0].valid);
    r.targets[8].seen=43.4;control={fresh};memory.update(r,control,{.02,0,0},43.5,true);check(control[0].valid);
    // Even a reset/initial display cache cannot advertise inhibited data.
    memory.reset();r.lidar_inhibited=true;control={fresh};memory.update(r,control,{},44,false);check(!control[0].valid);
    AssignmentHealth health;health.start();
    health.update(true,true,true,true,true,false,false,false);check(health.state==2);
    health.observation_paused();health.update(true,true,false,false,true,false,true,false);check(health.state==4);
    health.update(true,true,true,true,true,false,false,false);
    check(health.state==3&&health.reason==c2::ipc::Result::alignment_required);
    health.start();health.update(true,true,true,true,true,false,false,false);check(health.state==2);
    // Ordinary short data gaps still recover without an optical-pause latch.
    health.update(true,true,false,false,true,false,false,false);
    health.update(true,true,true,true,true,false,false,false);check(health.state==2);
    // ABI extension cannot accidentally advertise retained points as valid.
    c2::ipc::Frame frame;frame.payload.target_count=1;frame.payload.targets[0].id=7;
    frame.payload.targets[0].status=4;frame.payload.targets[0].x=1;
    c2::ipc::Frame decoded;check(c2::ipc::deserialize(c2::ipc::serialize(frame),decoded));
    frame.payload.targets[0].valid=1;check(!c2::ipc::deserialize(c2::ipc::serialize(frame),decoded));
    Session session;session.connect(0);auto hello=session.take().at(0);
    auto reply=hello;word(reply,0,0xDE01);word(reply,2,1);word(reply,4,word(hello,2));word(reply,20,0x207);
    session.receive(reply,.01);
    auto set=[](auto& p,unsigned n,uint16_t v){p[n]=uint8_t(v>>8);p[n+1]=uint8_t(v);};
    std::array<uint8_t,32> packet{};set(packet,0,0xDE03);set(packet,4,word(hello,2));
    set(packet,22,1800);set(packet,24,1);set(packet,26,1);
    for(uint16_t id=1;id<=10;++id){set(packet,2,id+1);set(packet,6,id);session.receive(packet,.1);}
    set(packet,2,12);set(packet,6,11);session.receive(packet,.2);
    check(session.state().targets.size()==10&&!session.state().targets.contains(11)); // Never evict a live track.
    set(packet,2,13);packet[28]=1;session.receive(packet,4);
    check(!session.state().targets.contains(11)); // Malformed data cannot consume a slot.
    packet[28]=0;session.receive(packet,4);
    check(session.state().targets.size()==10&&session.state().targets.contains(11)&&!session.state().targets.contains(1));
    return 0;
}
