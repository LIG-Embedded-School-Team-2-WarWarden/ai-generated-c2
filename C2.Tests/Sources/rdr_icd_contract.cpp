#include "c2/rdr_session.hpp"
#include <iostream>
#include <stdexcept>

// Independent RDR-ECS v0.7 local1 wire examples, transcribed from the received
// c2-lidar-complete board/zybo/rdr_ecs/{protocol.hpp,de03_sender.h} contract.
// Received protocol.hpp SHA256:
// 92e1f1fa2baf501100df8cee4e237e583d5a317afded1b2414c02a9d32730e54
// tests/rdr_board.hpp is the matching board fixture. These examples deliberately
// use neither that fixture nor Session's word()/make() helpers to construct bytes.
// There is no dependency on the received folder, sockets, or physical hardware.
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

    // DE03 has signed XYZ at 8/10/12, TILT at 14, PAN at 16, detected at 18.
    // Its offset 14 is not the control-result field, and it has no token field.
    client.receive(Packet{
        0xde,0x03, 0,5, 0x44,0x51, 0,7, 0xfe,0x70, 0,0x7d,
        0xf7,0x68, 0x80,0, 0xff,0x6f, 0,1, 0,0, 0,0}, .5);
    const auto& target = client.state().targets.at(7);
    check(target.status == 0 && target.x == -.4 && target.y == .125 && target.z == -2.2,
          "DE03 signed millimetres and detection flag must decode independently");
    client.receive(Packet{
        0xde,0x03, 0,6, 0x44,0x51, 0,7, 0,0, 0,0,
        0,0, 0x80,0, 0x80,0, 0,0, 0,0, 0,0}, .6);
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
    std::cout << "PASS: received RDR local1 literal 24-byte C2 contract\n";
return 0;
}
