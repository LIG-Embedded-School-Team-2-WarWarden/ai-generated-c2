#include "c2/rdr_session.hpp"
#include <iostream>
#include <stdexcept>

// Independent literal fixtures from Confluence RDR-ECS ICD page 37355617,
// revision 6 (2026-11-05): controls 24 bytes, DE03 32 bytes / float32 metres.
// User confirmed all RDR angles use 0.1 degrees despite remaining prose conflicts.
using c2::rdr::Packet;
using c2::rdr::Session;

static void check(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}
static void outgoing(Session& client, const Packet& expected, const char* message) {
    const auto packets = client.take();
    check(packets.size() == 1 && packets.front() == expected, message);
}

int main() {
    Session client(0x12345678);
    check(client.take().empty(), "construction must not emit traffic");
    check(client.connect(0) == 1, "connect must queue ED01");
    // Fresh token 0x12345679; initial request sequence 0x444e.
    outgoing(client, Packet{
        0xed,0x01, 0x44,0x4e, 0,0, 0,0, 0,0, 0x12,0x34,
        0x56,0x79, 0,0, 0,0, 0,0, 0,0, 0,0}, "ED01 literal bytes");
    client.receive(Packet{
        0xde,0x01, 0,1, 0x44,0x4e, 0,0, 0,0, 0x12,0x34,
        0x56,0x79, 0,0, 0x80,0, 0x80,0, 0,1, 0,0}, .01);
    check(client.state().connected && client.state().mode == 0,
          "literal DE01 must connect in IDLE");

    check(client.command(0xed03, 1, 0, .1) == 1, "report must queue ED03");
    outgoing(client, Packet{
        0xed,0x03, 0x44,0x4f, 0,0, 0,1, 0,0, 0x12,0x34,
        0x56,0x79, 0,0, 0,0, 0,0, 0,0, 0,0}, "ED03 literal bytes");
    check(client.state().last_result == 1 && !client.state().reporting,
          "ED03 requires a later DE02, not an invented direct ACK");
    client.tick(.2);
    outgoing(client, Packet{
        0xed,0x02, 0x44,0x50, 0,0, 0,0, 0,0, 0x12,0x34,
        0x56,0x79, 0,0, 0,0, 0,0, 0,0, 0,0}, "ED02 literal bytes");
    client.receive(Packet{
        0xde,0x02, 0,2, 0x44,0x50, 0,2, 0,0, 0x12,0x34,
        0x56,0x79, 0,0, 0x80,0, 0x80,0, 0,5, 0,0}, .21);
    check(client.state().reporting && client.state().last_result == 0 &&
          client.state().motor == 2 && client.state().pan == -32768,
          "literal DE02 must confirm reporting and unknown pose");

    check(client.command(0xed06, 1, 0, .3) == 1, "start must queue ED06");
    outgoing(client, Packet{
        0xed,0x06, 0x44,0x51, 0,0, 0,1, 0,0, 0x12,0x34,
        0x56,0x79, 0,0, 0,0, 0,0, 0,0, 0,0}, "ED06 start literal bytes");
    client.receive(Packet{
        0xde,0x06, 0,3, 0x44,0x51, 0,1, 0,0, 0x12,0x34,
        0x56,0x79, 0,1, 0x80,0, 0x80,0, 1,5, 0,0}, .31);
    check(client.state().mode == 1 && client.state().last_result == 1,
          "PENDING is not start completion");
    client.receive(Packet{
        0xde,0x06, 0,4, 0x44,0x51, 0,1, 0,0, 0x12,0x34,
        0x56,0x79, 0,0, 0x80,0, 0x80,0, 2,7, 0,0}, .4);
    check(client.state().mode == 2 && client.state().last_result == 0,
          "final DE06 must complete start in ACTIVE");

    // Literal big-endian float32: -0.4, 0.125, -2.2 metres.
    const std::array<uint8_t,32> observation{
        0xde,0x03, 0,5, 0x44,0x51, 0,7,
        0xbe,0xcc,0xcc,0xcd, 0x3e,0,0,0, 0xc0,0x0c,0xcc,0xcd,
        0x80,0, 0xff,0x6f, 0,1, 0,0,0,0,0,0};
    // Invalid packets must not create a target or consume its sequence number.
    for(size_t length : {size_t(0),size_t(5),size_t(24),size_t(31)})
        client.receive(std::span(observation).first(length), .45);
    for(size_t offset=26;offset<32;++offset){
        auto bad=observation;bad[offset]=1;client.receive(bad,.45);
    }
    for(size_t offset : {size_t(8),size_t(12),size_t(16)}){
        auto bad=observation;bad[offset]=0x7f;bad[offset+1]=0x80;
        bad[offset+2]=bad[offset+3]=0;client.receive(bad,.45); // infinity
        bad[offset+2]=1;client.receive(bad,.45); // NaN
    }
    auto bad_detected=observation;bad_detected[25]=2;client.receive(bad_detected,.45);
    check(client.state().targets.empty(),"reject wrong lengths, padding, nonfinite coordinates and invalid detection");
    client.receive(observation, .5);
    const auto& target = client.state().targets.at(7);
    check(target.status == 0 && std::abs(target.x+.4)<1e-6 && target.y == .125 && std::abs(target.z+2.2)<1e-6,
          "DE03 float32 metre coordinates and detection flag decode independently");
    auto missing=observation;missing[3]=6;
    std::fill(missing.begin()+8,missing.begin()+20,0);
    missing[20]=missing[22]=0x80;missing[21]=missing[23]=0;missing[25]=0;
    client.receive(missing,.6);
    check(target.status == 2 && target.x == 0 && target.y == 0 && target.z == 0,
          "DE03 detected=0 must invalidate the measurement");

    check(client.command(0xed04, 7, 0, .7) == 1, "exclude must queue ED04");
    outgoing(client, Packet{
        0xed,0x04, 0x44,0x52, 0,0, 0,7, 0,0, 0x12,0x34,
        0x56,0x79, 0,0, 0,0, 0,0, 0,0, 0,0}, "ED04 exclude literal bytes");
    client.receive(Packet{
        0xde,0x04, 0,7, 0x44,0x52, 0,7, 0,0, 0x12,0x34,
        0x56,0x79, 0,0, 0x80,0, 0x80,0, 2,7, 0,0}, .71);
    check(client.state().excluded.count(7) && target.status == 3,
          "DE04 must confirm exclusion");
    check(client.command(0xed04, 7, 1, .8) == 1, "resume must queue ED04");
    outgoing(client, Packet{
        0xed,0x04, 0x44,0x53, 0,0, 0,7, 0,1, 0x12,0x34,
        0x56,0x79, 0,0, 0,0, 0,0, 0,0, 0,0}, "ED04 resume literal bytes");
    client.receive(Packet{
        0xde,0x04, 0,8, 0x44,0x53, 0,7, 0,1, 0x12,0x34,
        0x56,0x79, 0,0, 0x80,0, 0x80,0, 2,7, 0,0}, .81);
    check(!client.state().excluded.count(7) && target.status == 2,
          "DE04 must confirm resume without inventing an observation");

    check(client.command(0xed06, 0, 0, .9) == 1, "stop must queue ED06");
    outgoing(client, Packet{
        0xed,0x06, 0x44,0x54, 0,0, 0,0, 0,0, 0x12,0x34,
        0x56,0x79, 0,0, 0,0, 0,0, 0,0, 0,0}, "ED06 stop literal bytes");
    client.receive(Packet{
        0xde,0x06, 0,9, 0x44,0x54, 0,0, 0,0, 0x12,0x34,
        0x56,0x79, 0,1, 0x80,0, 0x80,0, 3,5, 0,0}, .91);
    check(client.state().mode == 3 && client.state().last_result == 1,
          "stop must remain pending until final DE06");
    client.receive(Packet{
        0xde,0x06, 0,0x0a, 0x44,0x54, 0,0, 0,0, 0x12,0x34,
        0x56,0x79, 0,0, 0x80,0, 0x80,0, 0,5, 0,0}, 1);
    check(client.state().mode == 0 && client.state().last_result == 0,
          "final DE06 must complete stop in IDLE");
    std::cout << "PASS: RDR ICD 24-byte controls, 32-byte float32 DE03 and malformed data rejection\n";
return 0;
}
