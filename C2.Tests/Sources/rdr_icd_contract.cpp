#include "c2/rdr_session.hpp"
#include <iostream>
#include <stdexcept>

// Controls retain the literal ICD revision 6 fixtures. DE03 fixtures follow
// the user-confirmed 2026-11-05 raw-XYZ/observation-pose/TrackState contract;
// these bytes do not imply that the online ICD or device firmware was verified.
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

    // Literal raw sensor XYZ: -0.4, 0.125, -2.2 metres; TILT -12.3 degrees,
    // PAN 180.0 degrees, detected=1, TrackState=OBSERVED (1).
    const std::array<uint8_t,32> observation{
        0xde,0x03, 0,5, 0x44,0x51, 0,7,
        0xbe,0xcc,0xcc,0xcd, 0x3e,0,0,0, 0xc0,0x0c,0xcc,0xcd,
        0xff,0x85, 0x07,0x08, 0,1, 0,1, 0,0,0,0};
    // Invalid packets must not create a target or consume its sequence number.
    for(size_t length : {size_t(0),size_t(5),size_t(24),size_t(31)})
        client.receive(std::span(observation).first(length), .45);
    for(size_t offset=28;offset<32;++offset){
        auto bad=observation;bad[offset]=1;client.receive(bad,.45);
    }
    for(size_t offset : {size_t(8),size_t(12),size_t(16)}){
        auto bad=observation;bad[offset]=0x7f;bad[offset+1]=0x80;
        bad[offset+2]=bad[offset+3]=0;client.receive(bad,.45); // infinity
        bad[offset+2]=1;client.receive(bad,.45); // NaN
    }
    for(const auto& [detected,track_state] : std::array<std::pair<uint8_t,uint8_t>,6>{
            {{2,1},{0,1},{1,0},{1,2},{0,3},{0,255}}}){
        auto bad=observation;bad[25]=detected;bad[27]=track_state;client.receive(bad,.45);
    }
    auto wide_state=observation;wide_state[26]=1;client.receive(wide_state,.45);
    // LOST must carry zero XYZ and unknown pose, not stale measurements.
    auto malformed_lost=observation;malformed_lost[25]=malformed_lost[27]=0;
    client.receive(malformed_lost,.45);
    std::fill(malformed_lost.begin()+8,malformed_lost.begin()+20,0);
    client.receive(malformed_lost,.45);
    check(client.state().targets.empty(),"reject malformed size, reserved bytes, coordinates and state/detection combinations");
    client.receive(observation, .5);
    const auto& target = client.state().targets.at(7);
    check(target.status == 0 && target.detected == 1 && target.track_state == 1 &&
          std::abs(target.x+.4)<1e-6 && target.y == .125 && std::abs(target.z+2.2)<1e-6 &&
          target.tilt == -123 && target.pan == 1800,
          "OBSERVED preserves raw float32 metres and signed observation pose");
    auto predicted=observation;predicted[3]=6;predicted[25]=0;predicted[27]=2;
    // Unknown observation pose remains unknown even for a usable raw track.
    predicted[20]=predicted[22]=0x80;predicted[21]=predicted[23]=0;
    client.receive(predicted,.55);
    check(target.status == 0 && target.detected == 0 && target.track_state == 2 &&
          std::abs(target.x+.4)<1e-6 && target.y == .125 && std::abs(target.z+2.2)<1e-6 &&
          target.tilt == -32768 && target.pan == -32768,
          "PREDICTED keeps the same ID and valid raw XYZ despite detected=0; unknown pose is preserved");
    client.receive(observation,.56);
    check(target.track_state == 2 && target.seen == .55,"stale observation must not replace newer prediction");
    auto missing=observation;missing[3]=7;
    std::fill(missing.begin()+8,missing.begin()+20,0);
    missing[20]=missing[22]=0x80;missing[21]=missing[23]=0;missing[25]=missing[27]=0;
    client.receive(missing,.6);
    check(target.status == 2 && target.x == 0 && target.y == 0 && target.z == 0 &&
          target.detected == 0 && target.track_state == 0 && target.pan == -32768 && target.tilt == -32768,
          "LOST clears XYZ, pose, detection and track state");
    client.receive(predicted,.61);
    check(target.track_state == 0 && target.seen == .6,"stale prediction cannot revive a lost track");

    check(client.command(0xed04, 7, 0, .7) == 1, "exclude must queue ED04");
    outgoing(client, Packet{
        0xed,0x04, 0x44,0x52, 0,0, 0,7, 0,0, 0x12,0x34,
        0x56,0x79, 0,0, 0,0, 0,0, 0,0, 0,0}, "ED04 exclude literal bytes");
    client.receive(Packet{
        0xde,0x04, 0,8, 0x44,0x52, 0,7, 0,0, 0x12,0x34,
        0x56,0x79, 0,0, 0x80,0, 0x80,0, 2,7, 0,0}, .71);
    check(client.state().excluded.count(7) && target.status == 3,
          "DE04 must confirm exclusion");
    check(client.command(0xed04, 7, 1, .8) == 1, "resume must queue ED04");
    outgoing(client, Packet{
        0xed,0x04, 0x44,0x53, 0,0, 0,7, 0,1, 0x12,0x34,
        0x56,0x79, 0,0, 0,0, 0,0, 0,0, 0,0}, "ED04 resume literal bytes");
    client.receive(Packet{
        0xde,0x04, 0,9, 0x44,0x53, 0,7, 0,1, 0x12,0x34,
        0x56,0x79, 0,0, 0x80,0, 0x80,0, 2,7, 0,0}, .81);
    check(!client.state().excluded.count(7) && target.status == 2,
          "DE04 must confirm resume without inventing an observation");

    check(client.command(0xed06, 0, 0, .9) == 1, "stop must queue ED06");
    outgoing(client, Packet{
        0xed,0x06, 0x44,0x54, 0,0, 0,0, 0,0, 0x12,0x34,
        0x56,0x79, 0,0, 0,0, 0,0, 0,0, 0,0}, "ED06 stop literal bytes");
    client.receive(Packet{
        0xde,0x06, 0,0x0a, 0x44,0x54, 0,0, 0,0, 0x12,0x34,
        0x56,0x79, 0,1, 0x80,0, 0x80,0, 3,5, 0,0}, .91);
    check(client.state().mode == 3 && client.state().last_result == 1,
          "stop must remain pending until final DE06");
    client.receive(Packet{
        0xde,0x06, 0,0x0b, 0x44,0x54, 0,0, 0,0, 0x12,0x34,
        0x56,0x79, 0,0, 0x80,0, 0x80,0, 0,5, 0,0}, 1);
    check(client.state().mode == 0 && client.state().last_result == 0,
          "final DE06 must complete stop in IDLE");
    std::cout << "PASS: RDR ICD 24-byte controls, raw XYZ/pose/TrackState DE03 and malformed data rejection\n";
return 0;
}
