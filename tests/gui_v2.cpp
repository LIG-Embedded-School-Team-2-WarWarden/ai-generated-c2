#include "bridge.hpp"
#include <bit>
#include <fstream>
#include <iostream>
#include <limits>
using namespace c2mvp;
namespace w = c2::gui;
void check(bool ok, const char* message) { if (!ok) throw std::runtime_error(message); }
struct Time : Clock {
    double time() const override { return 1700000000; }
    double monotonic() const override { return 100; }
};
struct Transport : DatagramTransport {
    void send(const std::string&, const Endpoint&) override {}
    std::optional<Datagram> receive() override { return {}; }
    Endpoint address() const override { return {"127.0.0.1", 1}; }
};
int main(int argc, char** argv) {
    try {
        check(argc == 2, "config argument required");
        std::ifstream input(argv[1]);
        auto config = Config::parse(Json::parse(input));
        Time clock; Transport transport; LidarDetector detector; RelativePointingSolver solver;
        Engine engine(config, clock, transport, detector, solver, false);
        GuiBridge bridge(engine, false);
        auto& peer = engine.devices.peers[config.observer];
        peer.id = config.observer; peer.session = "test"; peer.alive = 100;
        peer.status.emplace(); peer.status->mutable_angles()->set_tilt_deg(15);
        for (const auto& [pan, expected] : std::array<std::pair<int,int>, 5>{{{-135,270},{-90,225},{0,135},{90,45},{135,0}}}) {
            peer.status->mutable_angles()->set_pan_deg(pan);
            auto f = bridge.frame();
            check(f.payload.rdr.azimuth == expected && f.payload.rdr.elevation == 15, "signed/GUI angle conversion");
        }
        auto f = bridge.frame();
        f.payload.rdr.position_x = 1.25f; f.payload.rdr.position_y = -2.5f; f.payload.rdr.field_of_view = 30;
        f.payload.fcs.position_x = -3.5f; f.payload.fcs.position_y = 4.25f;
        auto bytes = w::serialize(f);
        check(bytes.size() == 484 && bytes[4] == 2, "v2 header");
        // Independent golden IEEE-754 field bytes at the published C# offsets.
        check(bytes[58] == 0xa0 && bytes[59] == 0x3f, "RDR positionX offset/float encoding");
        check(bytes[62] == 0x20 && bytes[63] == 0xc0, "RDR positionY offset/float encoding");
        check(bytes[66] == 0xf0 && bytes[67] == 0x41, "RDR FOV offset/float encoding");
        check(bytes[98] == 0x60 && bytes[99] == 0xc0, "FCS positionX offset/float encoding");
        w::Frame decoded;
        check(w::deserialize(bytes, decoded) && decoded.payload.fcs.position_y == 4.25f, "v2 decode");
        for (size_t n = 0; n < bytes.size(); ++n)
            check(!w::deserialize(std::span(bytes).first(n), decoded), "truncated frame accepted");
        bytes[4] = 1;
        check(!w::deserialize(bytes, decoded), "v1 accepted as v2");
        bytes = w::serialize(f); bytes.back() ^= 1;
        check(!w::deserialize(bytes, decoded), "bad checksum accepted");
        f.payload.rdr.field_of_view = std::numeric_limits<float>::quiet_NaN();
        check(!w::deserialize(w::serialize(f), decoded), "nonfinite new field accepted");
        w::PipeCommand q; q.command = 4; q.request_id = 77; q.rdr_id = q.fcs_id = 1; q.target_id = 15;
        auto request = w::serialize(q); w::PipeCommand parsed;
        check(request.size() == 28 && request[8] == 4 && w::deserialize(request, parsed), "new command codec");
        q.target_id = 91;
        auto response = bridge.command(q);
        check(response.result == 1 && response.request_id == 77 && response.command == 4, "elevation bounds/correlation");
        q.target_id = 15;
        check(bridge.command(q).result == 2, "unavailable OBS command accepted");
        q.command = 99;
        check(bridge.command(q).result == 5, "unknown command accepted");
        std::cout << "GUI v2: layout, golden offsets, 484 truncations, checksum, new fields, angles, command validation passed\n";
    } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
