#include "c2/fcs_protocol.hpp"

#include <bit>
#include <cmath>
#include <stdexcept>

namespace c2::fcs {
std::int16_t angle_to_wire(double degrees) {
    const double scaled = degrees * 10.0;
    if (!std::isfinite(scaled) || scaled < -32767 || scaled > 32767 ||
        std::abs(scaled - std::round(scaled)) > 1e-8)
        throw std::invalid_argument("FCS angle requires 0.1 degree increments in wire range");
    return static_cast<std::int16_t>(std::round(scaled));
}
double angle_from_wire(std::int16_t value) noexcept { return value / 10.0; }
namespace {
void put(std::vector<std::byte>& bytes, std::uint16_t value) {
    bytes.push_back(static_cast<std::byte>(value >> 8));
    bytes.push_back(static_cast<std::byte>(value & 0xff));
}
std::uint16_t get(std::span<const std::byte> bytes, std::size_t offset) {
    return static_cast<std::uint16_t>((std::to_integer<unsigned>(bytes[offset]) << 8) |
        std::to_integer<unsigned>(bytes[offset + 1]));
}
std::size_t payload_end(MessageId id) {
    switch (id) {
        case MessageId::connect: case MessageId::status: case MessageId::demo: return 6;
        case MessageId::point: case MessageId::laser_reply: return 10;
        case MessageId::status_reply: return 12;
        default: return 8;
    }
}
void validate(const Packet& p) {
    (void)wire_size(p.id);
    if (!is_response(p.id) && p.reply_to != 0)
        throw std::invalid_argument("FCS requests must have zero reply-to sequence");
    if (is_response(p.id) && p.id != MessageId::status_reply &&
        static_cast<unsigned>(p.result) > 3)
        throw std::invalid_argument("undefined FCS result");
    if ((p.id == MessageId::laser || p.id == MessageId::laser_reply) && p.laser_state > 1)
        throw std::invalid_argument("invalid FCS laser state");
}
}  // namespace

bool is_response(MessageId id) noexcept {
    switch (id) {
        case MessageId::connect_reply: case MessageId::status_reply:
        case MessageId::point_reply: case MessageId::laser_reply:
        case MessageId::demo_reply: return true;
        default: return false;
    }
}
MessageId response_id(MessageId request) {
    switch (request) {
        case MessageId::connect: return MessageId::connect_reply;
        case MessageId::status: return MessageId::status_reply;
        case MessageId::point: return MessageId::point_reply;
        case MessageId::laser: return MessageId::laser_reply;
        case MessageId::demo: return MessageId::demo_reply;
        default: throw std::invalid_argument("not an FCS request ID");
    }
}
std::size_t wire_size(MessageId id) {
    switch (id) {
        case MessageId::connect: case MessageId::status: case MessageId::laser:
        case MessageId::demo: case MessageId::connect_reply:
        case MessageId::point_reply: case MessageId::demo_reply: return 8;
        case MessageId::point: case MessageId::status_reply:
        case MessageId::laser_reply: return 12;
        default: throw std::invalid_argument("unknown FCS message ID");
    }
}
std::vector<std::byte> encode(const Packet& p) {
    validate(p);
    std::vector<std::byte> bytes;
    bytes.reserve(wire_size(p.id));
    put(bytes, static_cast<std::uint16_t>(p.id));
    put(bytes, p.sequence);
    put(bytes, p.reply_to);
    switch (p.id) {
        case MessageId::point:
            put(bytes, std::bit_cast<std::uint16_t>(p.elevation));
            put(bytes, std::bit_cast<std::uint16_t>(p.azimuth)); put(bytes, 0); break;
        case MessageId::status_reply:
            put(bytes, p.flags);
            put(bytes, std::bit_cast<std::uint16_t>(p.elevation));
            put(bytes, std::bit_cast<std::uint16_t>(p.azimuth)); break;
        case MessageId::laser: put(bytes, p.laser_state); break;
        case MessageId::laser_reply:
            put(bytes, static_cast<std::uint16_t>(p.result));
            put(bytes, p.laser_state); put(bytes, 0); break;
        default:
            put(bytes, is_response(p.id) ? static_cast<std::uint16_t>(p.result) : 0);
    }
    return bytes;
}
Packet decode(std::span<const std::byte> bytes) {
    if (bytes.size() < 2) throw std::invalid_argument("truncated FCS header");
    Packet p;
    p.id = static_cast<MessageId>(get(bytes, 0));
    if (bytes.size() != wire_size(p.id))
        throw std::invalid_argument("incorrect FCS datagram length");
    for (auto offset = payload_end(p.id); offset < bytes.size(); ++offset)
        if (bytes[offset] != std::byte{})
            throw std::invalid_argument("nonzero FCS padding");
    p.sequence = get(bytes, 2); p.reply_to = get(bytes, 4);
    switch (p.id) {
        case MessageId::point:
            p.elevation = std::bit_cast<std::int16_t>(get(bytes, 6));
            p.azimuth = std::bit_cast<std::int16_t>(get(bytes, 8)); break;
        case MessageId::status_reply:
            p.flags = get(bytes, 6);
            p.elevation = std::bit_cast<std::int16_t>(get(bytes, 8));
            p.azimuth = std::bit_cast<std::int16_t>(get(bytes, 10)); break;
        case MessageId::laser: p.laser_state = get(bytes, 6); break;
        case MessageId::laser_reply:
            p.result = static_cast<Result>(get(bytes, 6));
            p.laser_state = get(bytes, 8); break;
        default:
            if (is_response(p.id)) p.result = static_cast<Result>(get(bytes, 6));
    }
    validate(p);
    return p;
}
}  // namespace c2::fcs
