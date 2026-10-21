#include "c2mvp/domain.hpp"
#include <iostream>
#include <stdexcept>
using namespace c2mvp;
void check(bool ok, const char* message) { if (!ok) throw std::runtime_error(message); }
struct FakeClock : Clock {
    double wall=1700000000, mono=100;
    double time() const override { return wall; }
    double monotonic() const override { return mono; }
    void advance(double s) { wall+=s; mono+=s; }
};
struct FakeTransport : DatagramTransport {
    std::vector<std::string> packets;
    void send(const std::string& data, const Endpoint&) override { packets.push_back(data); }
    std::optional<Datagram> receive() override { return {}; }
    Endpoint address() const override { return {"127.0.0.1",54321}; }
};
int main() {
    try {
        FakeClock clock; FakeTransport transport;
        RequestTracker requests(clock,transport);
        Peer peer; peer.id="201"; peer.session=uid(); peer.address={"127.0.0.1",5001};
        pb::Envelope cmd; auto* q=cmd.mutable_pointing_command();
        q->set_request_id(uid()); q->set_control_session_id(uid());
        q->set_control_epoch(2); q->set_command_sequence(7);
        q->set_operation(pb::PointingCommand::OPERATION_POINT);
        pb::Envelope reply; auto* r=reply.mutable_pointing_result();
        r->mutable_outcome()->set_request_id(q->request_id());
        r->mutable_outcome()->set_result(pb::REQUEST_RESULT_COMPLETED);
        r->mutable_outcome()->set_reported_at(clock.utc());
        r->mutable_outcome()->set_completed_at(clock.utc());
        r->set_control_session_id(q->control_session_id()); r->set_control_epoch(2);
        r->set_command_sequence(7);
        int completed=0;
        requests.request(peer,cmd,pb::Envelope::kPointingResult,[&](const pb::Envelope* e){if(e)++completed;});
        q->set_command_sequence(999);
        requests.result(peer,reply);
        check(completed==1,"caller mutation must not alter in-flight correlation");
        requests.result(peer,reply);
        check(completed==1,"duplicate result must not complete twice");
        transport.packets.clear(); int timeouts=0;
        requests.request(peer,cmd,pb::Envelope::kPointingResult,[&](const pb::Envelope* e){if(!e)++timeouts;});
        std::map<std::string,Peer> peers{{peer.id,peer}};
        for(int i=0;i<3;++i){clock.advance(.201);requests.tick(peers);}
        check(transport.packets.size()==3,"retry budget is three sends");
        check(transport.packets[0]==transport.packets[1] && transport.packets[1]==transport.packets[2],"retry exact bytes");
        check(timeouts==1 && requests.size()==0,"timeout clears pending before callback");
        ObserverOperation op; op.phase=ObserverPhase::Moving;
        bool rejected=false;try{op.transition(ObserverPhase::Completed);}catch(const std::invalid_argument&){rejected=true;}
        check(rejected,"collection cannot skip confirmation");
        std::cout<<"native domain: initial Red/Green contracts passed\n";
    } catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
