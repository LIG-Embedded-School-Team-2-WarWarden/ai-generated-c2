#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace c2::fcs {
// firmware/icd-fcs: one 8/12-byte message per UDP datagram, big endian.
// ID = source asset (4 bits), destination asset (4 bits), message number (8 bits).
enum class MessageId : std::uint16_t {
    connect = 0xEF01, status = 0xEF02, point = 0xEF03,
    laser = 0xEF04, demo = 0xEF05,
    connect_reply = 0xFE01, status_reply = 0xFE02, point_reply = 0xFE03,
    laser_reply = 0xFE04, demo_reply = 0xFE05,
};
enum class Result : std::uint16_t { accepted, busy, invalid_value, device_error };
struct Packet {
    MessageId id{};
    std::uint16_t sequence{};  // All uint16 values, including 0, are valid.
    std::uint16_t reply_to{};  // 0 on requests; echoed request sequence on replies.
    Result result{};
    std::uint16_t flags{};
    // Wire angles are signed tenths of a degree; -32768 means unknown in status.
    std::int16_t elevation{};
    std::int16_t azimuth{};
    std::uint16_t laser_state{};
};
[[nodiscard]] bool is_response(MessageId id) noexcept;
[[nodiscard]] std::int16_t angle_to_wire(double degrees);
[[nodiscard]] double angle_from_wire(std::int16_t value) noexcept;
[[nodiscard]] MessageId response_id(MessageId request);
[[nodiscard]] std::size_t wire_size(MessageId id);
[[nodiscard]] std::vector<std::byte> encode(const Packet& packet);
// Throws invalid_argument on unknown IDs, incorrect sizes or invalid fields.
[[nodiscard]] Packet decode(std::span<const std::byte> bytes);
}  // namespace c2::fcs
