#include "c2/rdr_session.hpp"
#include "rdr_board.hpp"
#include <cassert>
#include <iostream>
struct Harness {
    c2::rdr::Session client{123};Rdr::Engine board;double now=0;int starts=0,stops=0;
    Rdr::Packet last{};
    std::array<uint8_t,32> last_observation{};
    Harness(){board.send_observation=[&](const auto&p){last_observation=p;client.receive(p,now);};board.send=[&](const Rdr::Packet&p){last=p;client.receive(p,now);};
        board.control=[&](bool start){start?++starts:++stops;return true;};}
    void pump(){for(unsigned i=0;i<10;++i){auto out=client.take();if(out.empty())return;for(const auto&p:out){assert(p.size()==24);board.receive(p,now);}}assert(false);}
    void tick(double time){now=time;client.tick(now);pump();board.tick(now);}
    void connect(){assert(client.take().empty());client.connect(now);pump();tick(now);assert(client.state().connected);assert(board.mode==Rdr::IDLE&&starts==0);}
    void start(){assert(client.command(0xED06,1,0,now)==1);pump();assert(client.state().last_result==1);assert(board.mode==Rdr::STARTING);board.complete(true,true);assert(client.state().mode==2&&client.state().last_result==0);}
    void reporting(){assert(client.command(0xED03,1,0,now)==1);pump();tick(now+1.1);assert(client.state().reporting&&client.state().last_result==0);}
    void observe(uint16_t id,bool detected=true){std::array<uint8_t,32> p{};Rdr::put(p,24,detected?1:0);auto coord=[&](unsigned n,float v){auto bits=std::bit_cast<uint32_t>(v);Rdr::put(p,n,uint16_t(bits>>16));Rdr::put(p,n+2,uint16_t(bits));};coord(8,-.463f);coord(12,-.106f);coord(16,2.232f);board.observation(p,id);}
};
static void stale_status_cannot_complete_reporting(){
    c2::rdr::Session client{42};
    client.connect(0);auto hello=client.take().at(0);
    auto reply=hello;c2::rdr::word(reply,0,0xDE01);
    c2::rdr::word(reply,2,10);c2::rdr::word(reply,4,c2::rdr::word(hello,2));
    c2::rdr::word(reply,20,5); // Connected, reporting already on, IDLE.
    client.receive(reply,0);assert(client.state().connected&&client.state().reporting);
    client.command(0xED03,1,0,0);client.take();
    client.tick(1.1);auto outgoing=client.take();
    auto poll=std::find_if(outgoing.begin(),outgoing.end(),[](const auto&p){return c2::rdr::word(p,0)==0xED02;});
    assert(poll!=outgoing.end());
    c2::rdr::word(reply,0,0xDE02);c2::rdr::word(reply,4,c2::rdr::word(*poll,2));
    client.receive(reply,1.1); // Correlated, but duplicate board sequence 10.
    assert(client.state().last_result==1);
    client.tick(2.2);outgoing=client.take();
    poll=std::find_if(outgoing.begin(),outgoing.end(),[](const auto&p){return c2::rdr::word(p,0)==0xED02;});
    assert(poll!=outgoing.end());c2::rdr::word(reply,2,11);c2::rdr::word(reply,4,c2::rdr::word(*poll,2));
    client.receive(reply,2.2);assert(client.state().last_result==0);
}
static void control_acknowledgements_without_echoed_arguments(){
    Harness h;h.connect();h.reporting();
    h.board.send=[&](const Rdr::Packet&p){
        auto reply=p;
        if(Rdr::get(reply,0)==0xDE04||Rdr::get(reply,0)==0xDE06){
            Rdr::put(reply,6,0);Rdr::put(reply,8,0);
        }
        h.last=reply;h.client.receive(reply,h.now);
    };
    // The verified C2 contract correlates controls by nonce, response MsgID
    // and RxSeq. Argument slots in replies need not echo their request.
    h.start();h.observe(7);
    assert(h.client.command(0xED04,7,0,h.now)==1);h.pump();
    assert(h.client.state().last_result==0&&h.client.state().excluded.count(7));
    assert(h.client.command(0xED04,7,1,h.now)==1);h.pump();
    assert(h.client.state().last_result==0&&!h.client.state().excluded.count(7));
    h.client.command(0xED06,0,0,h.now);h.pump();h.board.complete(false,true);
    assert(h.client.state().last_result==0&&h.client.state().mode==0);
}
static void retries_preserve_packet_after_newer_poll(){
    // Drop the first ED03, then let a newer ED02 reach the board before retry.
    Harness h; h.connect();
    h.client.command(0xED03,1,0,.2);
    const auto original=h.client.take().at(0);
    h.tick(1.05);
    assert(!h.board.reporting);
    h.now=1.3; h.client.tick(h.now);
    auto retry=h.client.take();
    assert(retry.size()==1 && retry.front()==original);
    h.board.receive(retry.front(),h.now);
    assert(!h.board.reporting && h.client.state().last_result==1);
    // Polls keep the session alive; no retry may invent a newer ED03 sequence.
    for(int second=2;second<=40;++second){
        h.now=second+.3; h.client.tick(h.now);
        for(const auto& packet:h.client.take()){
            if(c2::rdr::word(packet,0)==0xED03)assert(packet==original);
            h.board.receive(packet,h.now);
        }
        h.board.tick(h.now);
    }
    assert(h.client.state().connected && !h.board.reporting);
    assert(h.client.state().last_result==9); // Unconfirmed operation times out locally.
}
int main(){
    retries_preserve_packet_after_newer_poll();
    stale_status_cannot_complete_reporting();
    control_acknowledgements_without_echoed_arguments();
    Harness h;h.connect();h.reporting();h.start();h.observe(1);h.observe(2);
    auto s=h.client.state();assert(s.targets.size()==2&&std::abs(s.targets.at(1).x+.463)<1e-6&&std::abs(s.targets.at(1).y+.106)<1e-6&&std::abs(s.targets.at(1).z-2.232)<1e-6);
    h.observe(1,false);assert(h.client.state().targets.at(1).status==2&&h.client.state().targets.at(1).x==0);assert(h.client.state().targets.at(2).status==0);
    auto duplicate=h.last_observation;h.now+=.1;h.client.receive(duplicate,h.now);assert(h.client.state().targets.at(1).seen!=h.now);
    assert(h.client.command(0xED04,2,0,h.now)==1);
    auto exclude=h.client.take().at(0);auto wrong_request=exclude;
    Rdr::put(wrong_request,0,0xDE04);Rdr::put(wrong_request,2,100);
    Rdr::put(wrong_request,4,uint16_t(Rdr::get(exclude,2)+1));
    Rdr::put(wrong_request,14,0);Rdr::put(wrong_request,20,0x0207);
    h.client.receive(wrong_request,h.now);
    assert(h.client.state().excluded.empty()&&h.client.state().last_result==1);
    h.board.receive(exclude,h.now);assert(h.client.state().excluded.count(2)&&h.client.state().targets.at(2).status==3);
    h.client.command(0xED06,0,0,h.now);h.pump();h.board.complete(false,true);assert(h.client.state().mode==0);h.start();
    assert(h.client.state().excluded.count(2));h.client.command(0xED04,2,1,h.now);h.pump();assert(!h.client.state().excluded.count(2));
    h.observe(2);assert(h.client.state().targets.at(2).status==0);
    h.tick(h.now+3.1);assert(h.client.state().targets.at(2).status==2);
    Harness dropped;dropped.connect();dropped.client.command(0xED03,1,0,0);dropped.client.take(); // drop first ED03
    dropped.tick(1.1);assert(dropped.board.reporting);assert(dropped.client.state().last_result==0);
    Harness retry;retry.connect();retry.start();retry.client.command(0xED06,0,0,0);retry.pump();retry.tick(1.1);assert(retry.stops==1);retry.board.complete(false,true);
    // Reconnect clears IDs/exclusions and never automatically starts surveillance.
    const auto generation_before=h.client.state().generation;
    assert(generation_before==1);
    h.now+=4;h.board.tick(h.now);h.board.complete(false,true);h.client.tick(h.now);h.pump();
    assert(h.client.state().connected&&h.client.state().mode==0&&h.client.state().targets.empty());
    assert(h.client.state().generation==generation_before+1);
    h.tick(h.now+1.1);assert(h.client.state().generation==generation_before+1);
    assert(h.starts==2);
    assert(c2::rdr::newer(0,65535)&&!c2::rdr::newer(65535,0));
    auto wrong=h.last;Rdr::session(wrong,999);const auto before=h.client.state().mode;h.client.receive(wrong,h.now);assert(h.client.state().mode==before);
    Harness bounded;bounded.connect();bounded.reporting();bounded.start();
    for(uint16_t id=1;id<=11;++id)bounded.observe(id);
    assert(bounded.client.state().targets.size()==10&&bounded.client.state().targets.count(10));
    assert(!bounded.client.state().targets.count(11));
    bounded.observe(1,false);assert(bounded.client.state().targets.at(1).status==2);
    bounded.client.command(0xED04,11,0,bounded.now);bounded.pump();
    assert(bounded.client.state().targets.size()==10&&bounded.client.state().excluded.count(11));
    std::cout<<"PASS: 24-byte codec, idle connect, pending completion, per-ID aggregation/loss, exclusion/resume, stop/start, retry, dropped ED03, stale data, lease reconnect\n";
return 0;
}
