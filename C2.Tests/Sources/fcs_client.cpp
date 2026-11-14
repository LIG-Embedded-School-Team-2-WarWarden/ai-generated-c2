#include "c2/fcs_client.hpp"
#include "fixtures/firmware_protocol.h"

#include <array>
#include <atomic>
#include <chrono>
#include <future>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

using namespace std::chrono_literals;
using namespace c2::fcs;
namespace {
void check(bool ok, const char* message) {
    if (!ok) throw std::runtime_error(message);
}
template<class F> void rejects(F action) {
    bool threw = false;
    try { action(); } catch (const std::exception&) { threw = true; }
    check(threw, "expected rejection");
}
std::string firmware_bytes(const std::vector<std::byte>& bytes) {
    return {reinterpret_cast<const char*>(bytes.data()), bytes.size()};
}
std::vector<std::byte> ecs_bytes(const std::string& bytes) {
    const auto* begin = reinterpret_cast<const std::byte*>(bytes.data());
    return {begin, begin + bytes.size()};
}
ClientConfig config(c2::Endpoint peer) {
    ClientConfig c;
    c.peer = std::move(peer); c.local.bind_address = "127.0.0.1";
    c.response_timeout = 150ms;
    return c;
}
void codec_tests() {
    Packet point{MessageId::point, 0x1234};
    point.elevation = -10; point.azimuth = 180;
    const std::array<unsigned, 12> expected{0xef, 3, 0x12, 0x34, 0, 0, 0xff, 0xf6, 0, 0xb4, 0, 0};
    const auto bytes = encode(point);
    for (std::size_t i = 0; i < bytes.size(); ++i)
        check(std::to_integer<unsigned>(bytes[i]) == expected[i], "golden big-endian point bytes");
    check(decode(bytes).elevation == -10, "signed angle decode");
    constexpr std::array ids{
        MessageId::connect, MessageId::status, MessageId::point, MessageId::laser, MessageId::demo,
        MessageId::connect_reply, MessageId::status_reply, MessageId::point_reply,
        MessageId::laser_reply, MessageId::demo_reply};
    constexpr std::array<std::size_t, 10> sizes{8,8,12,8,8,8,12,8,12,8};
    for (std::size_t i = 0; i < ids.size(); ++i) {
        Packet p{ids[i], 1, static_cast<std::uint16_t>(is_response(ids[i]) ? 25 : 0)};
        p.flags = 0x1f; p.elevation = -32768; p.azimuth = 32767; p.laser_state = 1;
        const auto encoded = encode(p);
        check(encoded.size() == sizes[i], "message length");
        check(decode(encoded).id == ids[i], "round trip ID");
        for (std::size_t size = 0; size < encoded.size(); ++size)
            rejects([&] { (void)decode(std::span(encoded).first(size)); });
        auto long_packet = encoded; long_packet.push_back(std::byte{});
        rejects([&] { (void)decode(long_packet); });
    }
    rejects([] { (void)decode({}); });
    auto unknown = bytes; unknown[1] = std::byte{0xab};
    rejects([&] { (void)decode(unknown); });
    Packet invalid{MessageId::laser, 1}; invalid.laser_state = 2;
    rejects([&] { (void)encode(invalid); });
    invalid = {MessageId::point_reply, 1, 1}; invalid.result = static_cast<Result>(4);
    rejects([&] { (void)encode(invalid); });
    invalid = {MessageId::status, 1, 1};
    rejects([&] { (void)encode(invalid); });
    auto padding = encode(Packet{MessageId::connect, 1});
    padding[6] = std::byte{0xff};
    rejects([&] { (void)decode(padding); });
    auto little_endian = bytes;
    for (std::size_t i = 0; i < little_endian.size(); i += 2)
        std::swap(little_endian[i], little_endian[i + 1]);
    rejects([&] { (void)decode(little_endian); });
    rejects([] { (void)decode(ecs_bytes(icd::packet(1, 0, 0))); });

    // Use the actual pinned firmware codec as an independent interoperability oracle.
    for (const auto id : ids) for (const std::uint16_t seq : {0, 1, 65535}) {
        Packet p{id, seq, static_cast<std::uint16_t>(is_response(id) ? seq : 0)};
        p.result = Result::busy; p.flags = 0x1f;
        p.elevation = -32768; p.azimuth = 32767; p.laser_state = 1;
        const auto raw_id = static_cast<unsigned>(id);
        check(wire_size(id) == icd::length(raw_id), "firmware message length");
        std::vector<int> values;
        switch (id) {
            case MessageId::point: values = {p.elevation, p.azimuth}; break;
            case MessageId::laser: values = {p.laser_state}; break;
            case MessageId::status_reply: values = {p.flags, p.elevation, p.azimuth}; break;
            case MessageId::laser_reply: values = {int(p.result), p.laser_state}; break;
            default: if (is_response(id)) values = {int(p.result)};
        }
        const auto firmware = icd::packet(raw_id, seq, p.reply_to, values);
        check(firmware_bytes(encode(p)) == firmware, "ECS encoding equals firmware encoding");
        const auto decoded = decode(ecs_bytes(firmware));
        check(decoded.sequence == seq && decoded.reply_to == p.reply_to, "zero and boundary sequences");
        if (!is_response(id)) {
            check(icd::validRequest(firmware_bytes(encode(p))), "firmware accepts ECS request");
            check(static_cast<unsigned>(response_id(id)) == icd::responseId(raw_id), "firmware response ID");
        }
        if (id == MessageId::point || id == MessageId::status_reply)
            check(decoded.elevation == -32768 && decoded.azimuth == 32767, "firmware signed values");
        if (is_response(id) && id != MessageId::status_reply)
            check(decoded.result == Result::busy, "firmware result decode");
        if (id == MessageId::status_reply) check(decoded.flags == p.flags, "firmware flags decode");
        if (id == MessageId::laser || id == MessageId::laser_reply)
            check(decoded.laser_state == 1, "firmware state decode");
        const auto end = id == MessageId::point || id == MessageId::laser_reply ? 10u :
            id == MessageId::connect || id == MessageId::status || id == MessageId::demo ? 6u :
            static_cast<unsigned>(firmware.size());
        for (auto offset = end; offset < firmware.size(); ++offset) {
            auto corrupt = ecs_bytes(firmware); corrupt[offset] = std::byte{1};
            rejects([&] { (void)decode(corrupt); });
        }
    }
}
void client_tests() {
    std::atomic<bool> drop_status{}, drop_control{}, unknown_angles{};
    std::atomic<unsigned> control_count{};
    std::atomic<int> pointed_elevation{}, pointed_azimuth{};
    std::atomic<bool> inject_noise{};
    std::atomic<std::uint16_t> response_sequence{0}; // Firmware starts its replies at zero.
    c2::UdpTransport wrong_source({"127.0.0.1", 0}, [](auto, auto) {});
    wrong_source.start();
    c2::UdpTransport* server_ptr{};
    c2::UdpTransport server({"127.0.0.1", 0}, [&](auto bytes, auto source) {
        const auto wire = firmware_bytes(bytes);
        check(icd::validRequest(wire), "UDP request accepted by firmware codec");
        const auto id = icd::word(wire, 0);
        const auto seq = icd::word(wire, 2);
        const auto request = decode(bytes);
        if (request.id == MessageId::point) {
            pointed_elevation = static_cast<std::int16_t>(icd::word(wire, 6));
            pointed_azimuth = static_cast<std::int16_t>(icd::word(wire, 8));
        }
        if (request.id == MessageId::status && drop_status) return;
        if (request.id == MessageId::point || request.id == MessageId::laser || request.id == MessageId::demo) {
            ++control_count;
            if (drop_control) return;
        }
        std::vector<int> values = id == icd::EF02 ? std::vector<int>{7, -125, 453} :
            id == icd::EF04 ? std::vector<int>{icd::ACCEPTED, int(icd::word(wire, 6))} :
            std::vector<int>{icd::ACCEPTED};
        if (id == icd::EF02 && unknown_angles) values = {7, -32768, -32768};
        const auto encoded_reply = ecs_bytes(icd::packet(icd::responseId(id), response_sequence++, seq, values));
        if (inject_noise) {
            const auto reply = decode(encoded_reply);
            auto noise = reply;
            noise.result = Result::device_error;
            wrong_source.send(encode(noise), source);
            noise.id = request.id == MessageId::connect ? MessageId::demo_reply : MessageId::connect_reply;
            server_ptr->send(encode(noise), source);
            noise = reply; noise.reply_to = static_cast<std::uint16_t>(request.sequence + 1);
            server_ptr->send(encode(noise), source);
            server_ptr->send(std::array{std::byte{1}}, source);
            std::this_thread::sleep_for(10ms);
        }
        server_ptr->send(encoded_reply, source);
        server_ptr->send(encoded_reply, source); // duplicate response must not satisfy next request
    });
    server_ptr = &server;
    server.start();
    std::mutex diagnostic_mutex;
    std::string diagnostics;
    auto client_config=config(server.local_endpoint());
    client_config.diagnostic=[&](const std::string& line) {
        std::lock_guard lock(diagnostic_mutex);
        diagnostics+=line+'\n';
    };
    Client client(client_config);
    client.start();
    rejects([&] { (void)client.query(); });
    inject_noise = true;
    const auto connected = client.connect();
    if (!connected)
        std::cerr << "connect timeout: server rx=" << server.stats().received_datagrams
                  << " tx=" << server.stats().sent_datagrams
                  << " handler_errors=" << server.stats().handler_errors << '\n';
    check(connected && connected->result == Result::accepted, "match source, ID and reply-to");
    check(connected->sequence == 0, "accept firmware initial reply sequence zero");
    check(client.snapshot().connected, "connected after handshake");
    {
        std::lock_guard lock(diagnostic_mutex);
        for(const auto* expected : {"reason=unexpected_peer", "reason=unexpected_message_id",
            "reason=reply_to_mismatch", "reason=decode_error", "rx_match id=0xFE01",
            "tx id=0xEF01 sequence=1", "hex="})
            check(diagnostics.find(expected)!=std::string::npos, "wire diagnostic identifies injected failures");
    }
    inject_noise = false;
    const auto status = client.query();
    check(status && status->elevation == -125 && status->azimuth == 453, "raw status tenths of degrees");
    check(status->sequence == 2, "fresh reply after handshake status probe");
    check(client.snapshot().status.has_value(), "status snapshot");
    check(client.snapshot().status->elevation == -12.5 && client.snapshot().status->azimuth == 45.3,
          "status converted to degrees");
    const auto pointed = client.point(-10.5, 90.2);
    check(pointed && pointed->sequence == 3, "accept forward firmware reply sequence");
    check(pointed_elevation == -105 && pointed_azimuth == 902, "point degrees converted to wire tenths");
    const auto light = client.laser(true);
    check(light && light->laser_state == 1, "laser reply");
    check(client.demo().has_value(), "demo reply");
    unknown_angles = true;
    check(client.query().has_value() && client.snapshot().status->elevation == -32768 &&
          client.snapshot().status->azimuth == -32768, "unknown status angles preserved");
    unknown_angles = false;
    check(client.point(-7.9,150.4).has_value() && pointed_elevation==-79 && pointed_azimuth==1504,
          "negative tilt from live assignment is sent without ECS mechanical limits");
    check(client.point(99,400).has_value() && pointed_elevation==990 && pointed_azimuth==4000,
          "angles outside former installation limits reach the device");
    check(client.point(-3276.7,3276.7).has_value() && pointed_elevation==-32767 && pointed_azimuth==32767,
          "signed wire angle bounds remain supported");
    const auto before = control_count.load();
    rejects([&] { (void)client.point(3276.8, 90); });
    rejects([&] { (void)client.point(20, -3276.8); });
    rejects([&] { (void)client.point(20.55, 90); });
    rejects([&] { (void)client.point(std::numeric_limits<double>::quiet_NaN(), 90); });
    rejects([&] { (void)client.point(20, std::numeric_limits<double>::infinity()); });
    check(control_count == before, "invalid wire angles are not sent");
    drop_control = true;
    check(!client.demo(), "control timeout has unknown outcome");
    {
        std::lock_guard lock(diagnostic_mutex);
        check(diagnostics.find("timeout outcome_unknown id=0xEF05")!=std::string::npos,
            "timeout diagnostic identifies command");
        check(diagnostics.find("timeout_transport id=0xEF05")!=std::string::npos,
            "timeout includes transport error counters");
    }
    check(control_count == before + 1, "control command not retried");
    drop_status = true;
    check(!client.query() && client.snapshot().connected, "first status timeout");
    drop_status = false;
    check(client.query().has_value() && client.snapshot().missed_status == 0, "reset miss counter");
    drop_status = true;
    for (int i = 0; i < 3; ++i) (void)client.query();
    check(!client.snapshot().connected, "three misses disconnect");
    rejects([&] { (void)client.demo(); });
    check(!client.connect() && !client.snapshot().connected,
          "accepted handshake without fresh status is not a usable connection");
    rejects([&] { (void)client.demo(); });
    drop_status = false;
    response_sequence = 65534;
    check(client.connect().has_value() && client.snapshot().connected, "reconnect");
    check(client.query()->sequence == 0, "fresh reply sequence wraps through zero");
    drop_status = true;
    auto waiting = std::async(std::launch::async, [&] { return client.query(); });
    std::this_thread::sleep_for(20ms);
    client.stop();
    check(waiting.wait_for(1s) == std::future_status::ready, "stop wakes pending request");
    check(!waiting.get(), "stopped request has unknown outcome");
    server.stop(); wrong_source.stop();
}
}  // namespace
int main() {
    try {
        codec_tests();
        client_tests();
        std::cout << "FCS codec/client checks passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "FCS test failed: " << error.what() << '\n';
        return 1;
    }
return 0;
}
