#include "lidar_interlock.hpp"
#include <stdexcept>
static void require(bool b){if(!b)throw std::runtime_error("lidar interlock contract failed");}
int lidar_interlock_entry(){
    using namespace std::chrono;
    const auto now=LidarInterlock::Clock::now();
    c2::rdr::State r;r.connected=true;r.generation=1;r.mode=2;
    LidarInterlock gate;
    using Block=LidarInterlock::Block;
    require(gate.block_reason(r)==Block::unsupported);
    require(!gate.begin(r,now)); // Legacy board must not enable optical output.
    r.lidar_supported=true;
    require(gate.block_reason(r)==Block::unknown);
    r.lidar_known=true;r.connected=false;
    require(gate.block_reason(r)==Block::disconnected);
    r.connected=true;r.lidar_inhibited=true;
    require(gate.block_reason(r)==Block::recovery_required);
    r.lidar_inhibited=false;r.last_result=1;
    require(gate.block_reason(r)==Block::busy);
    r.last_result=0;
    require(gate.block_reason(r)==Block::none);
    r.lidar_supported=r.lidar_known=r.lidar_on=true;
    require(gate.begin(r,now));require(!gate.stopped(r));
    r.last_command=0xED07;r.last_result=r.lidar_result=1;r.lidar_inhibited=true;r.lidar_on=false;
    require(!gate.stopped(r));r.last_result=r.lidar_result=0;require(gate.stopped(r));
    gate.emitted(now);
    c2::fcs::Snapshot f;f.connected=true;f.status=c2::fcs::Status{1,0,0};
    f.status_received_at=now+milliseconds(10);
    require(!gate.update(r,f,now+milliseconds(20))); // Idle is not completion without evidence.
    f.status->flags=13;require(!gate.update(r,f,now+milliseconds(20)));
    f.status->flags=1;f.connected=false;require(!gate.update(r,f,now+milliseconds(20)));
    f.connected=true;require(!gate.update(r,f,now+seconds(3))); // Stale OFF is insufficient.
    f.status_received_at=now+seconds(3);
    require(gate.update(r,f,now+seconds(3)));
    require(!gate.update(r,f,now+seconds(3))); // Resume requested once, not every tick.
    r.lidar_request=1;r.lidar_inhibited=false;r.lidar_on=true;
    require(!gate.update(r,f,now+seconds(3)));require(gate.phase()==LidarInterlock::Phase::idle);
    require(gate.begin(r,now));gate.emitted(now);r.lidar_inhibited=true;r.lidar_on=false;
    ++r.generation;f.status->flags=5;f.status_received_at=now+milliseconds(1);
    require(!gate.update(r,f,now+milliseconds(2)));f.status->flags=1;
    require(!gate.update(r,f,now+milliseconds(2))); // Reconnect cannot restore automatically.
    c2::fcs::Packet off{c2::fcs::MessageId::laser_reply};off.result=c2::fcs::Result::accepted;
    gate.off_reply(off,now+milliseconds(3));require(!gate.update(r,f,now+milliseconds(4)));
    f.status_received_at=now+milliseconds(5);require(gate.update(r,f,now+milliseconds(6)));
    require(!gate.update(r,f,now+seconds(4)));require(gate.phase()==LidarInterlock::Phase::fault);

    // Real Session rejects malformed/old DE07 acknowledgements and retries the
    // original bytes. No status-only response can complete an ED07 operation.
    c2::rdr::Session s;require(s.connect(0)==1);auto hello=s.take().at(0);
    auto response=hello;c2::rdr::word(response,0,0xDE01);c2::rdr::word(response,2,1);
    c2::rdr::word(response,4,c2::rdr::word(hello,2));c2::rdr::word(response,20,0x239);
    s.receive(response,.01);require(s.command(0xED07,0,0,.1)==1);auto stop=s.take().at(0);
    response=stop;c2::rdr::word(response,0,0xDE07);c2::rdr::word(response,4,c2::rdr::word(stop,2));
    c2::rdr::word(response,2,2);c2::rdr::word(response,20,0x279); // ON+inhibited invalid success.
    s.receive(response,.2);require(s.state().last_result==1);
    c2::rdr::word(response,20,0x259);c2::rdr::word(response,6,1);
    s.receive(response,.3);require(s.state().last_result==1);
    c2::rdr::word(response,6,0);c2::rdr::word(response,2,1);
    s.receive(response,.4);require(s.state().last_result==1);
    s.tick(1.2);auto retry=s.take();require(retry.at(0)==stop);
    auto poll=retry.at(1);c2::rdr::word(poll,0,0xDE02);
    c2::rdr::word(poll,4,c2::rdr::word(poll,2));c2::rdr::word(poll,2,3);
    c2::rdr::word(poll,20,0x259);s.receive(poll,1.21);
    require(s.state().last_result==1); // Even latest OFF status needs a DE07 completion.
    c2::rdr::word(response,2,2);s.receive(response,1.3);
    require(s.state().last_result==0&&s.state().lidar_inhibited&&!s.state().lidar_on);
    // ED07 expires at two seconds, stops retransmitting, and permits a new
    // recovery request without waiting for the general 40-second timeout.
    c2::rdr::Session timeout;
    timeout.connect(0);auto connect=timeout.take().at(0);
    auto ack=connect;c2::rdr::word(ack,0,0xDE01);c2::rdr::word(ack,2,1);
    c2::rdr::word(ack,4,c2::rdr::word(connect,2));c2::rdr::word(ack,20,0x259);
    timeout.receive(ack,.01);require(timeout.command(0xED07,0,0,1)==1);
    const auto original=timeout.take().at(0);
    timeout.tick(2);require(timeout.take().at(0)==original);
    timeout.tick(3);
    require(timeout.state().lidar_result==9&&timeout.state().last_result==9);
    for(const auto& packet:timeout.take())require(c2::rdr::word(packet,0)!=0xED07);
    require(timeout.command(0xED07,1,0,3)==1);auto resume=timeout.take().at(0);
    require(c2::rdr::word(resume,2)!=c2::rdr::word(original,2));
    ack=original;c2::rdr::word(ack,0,0xDE07);c2::rdr::word(ack,2,2);
    c2::rdr::word(ack,4,c2::rdr::word(original,2));c2::rdr::word(ack,20,0x259);
    timeout.receive(ack,3.01);require(timeout.state().lidar_result==1);
    ack=resume;c2::rdr::word(ack,0,0xDE07);c2::rdr::word(ack,2,3);
    c2::rdr::word(ack,4,c2::rdr::word(resume,2));c2::rdr::word(ack,20,0x239);
    timeout.receive(ack,3.02);require(timeout.state().lidar_result==0);
    require(timeout.command(0xED07,0,0,4)==1);auto late=timeout.take().at(0);
    c2::rdr::word(late,0,0xDE07);c2::rdr::word(late,4,c2::rdr::word(late,2));
    c2::rdr::word(late,2,4);c2::rdr::word(late,20,0x259);
    timeout.receive(late,6);require(timeout.state().lidar_result==9);
    require(timeout.command(0xED07,1,0,6)==1);
    return 0;
}
