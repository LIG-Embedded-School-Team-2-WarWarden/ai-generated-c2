#include "protocol.hpp"

#include <bit>
#include <cmath>
#include <limits>

namespace c2::ipc {
namespace {
static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559);
struct Writer {
    std::vector<std::uint8_t> bytes;
    void operator()(std::uint8_t& v) { bytes.push_back(v); }
    template<class T> requires std::is_unsigned_v<T>
    void operator()(T& v) {
        for (std::size_t i = 0; i < sizeof(T); ++i)
            bytes.push_back(static_cast<std::uint8_t>(v >> (8 * i)));
    }
    void operator()(std::int32_t& v) { auto bits = std::bit_cast<std::uint32_t>(v); (*this)(bits); }
    void operator()(std::int16_t& v) { auto bits = std::bit_cast<std::uint16_t>(v); (*this)(bits); }
    void operator()(float& v) { auto bits = std::bit_cast<std::uint32_t>(v); (*this)(bits); }
};
struct Reader {
    std::span<const std::uint8_t> bytes;
    std::size_t offset{};
    template<class T> requires std::is_unsigned_v<T>
    void operator()(T& v) {
        v = 0;
        for (std::size_t i = 0; i < sizeof(T); ++i)
            v = static_cast<T>(v | (static_cast<std::uint64_t>(bytes[offset++]) << (8 * i)));
    }
    void operator()(std::int32_t& v) { std::uint32_t bits; (*this)(bits); v = std::bit_cast<std::int32_t>(bits); }
    void operator()(std::int16_t& v) { std::uint16_t bits; (*this)(bits); v = std::bit_cast<std::int16_t>(bits); }
    void operator()(float& v) { std::uint32_t bits; (*this)(bits); v = std::bit_cast<float>(bits); }
};
template<class IO, class... Values> void fields(IO& io, Values&... values) { (io(values), ...); }
template<class IO> void visit(IO& io, RdrAsset& v) {
    fields(io, v.id, v.enabled, v.operating, v.connected, v.azimuth, v.elevation,
           v.position_x, v.position_y, v.position_z, v.field_of_view);
}
template<class IO> void visit(IO& io, FcsAsset& v) {
    fields(io, v.id, v.enabled, v.operating, v.connected, v.laser_on, v.azimuth, v.elevation,
           v.position_x, v.position_y, v.position_z);
}
template<class IO> void visit(IO& io, Target& v) {
    fields(io, v.id, v.azimuth, v.elevation, v.distance, v.x, v.y, v.z,
           v.rdr_id, v.status, v.valid, v.reserved);
}
template<class IO> void visit(IO& io, Payload& v) {
    visit(io, v.rdr); visit(io, v.fcs);
    fields(io, v.rdr_count, v.fcs_count, v.selected_rdr, v.selected_fcs, v.target_count);
    for (auto& target : v.targets) visit(io, target);
}
template<class IO> void visit(IO& io, FrameHeader& v) {
    fields(io, v.magic, v.version, v.size, v.sequence, v.timestamp_ms, v.payload_bytes, v.checksum);
}
template<class IO> void visit(IO& io, Frame& v) { visit(io, v.header); visit(io, v.payload); }
template<class IO> void visit(IO& io, PipeCommand& v) {
    fields(io, v.magic, v.version, v.command, v.request_id, v.rdr_id, v.fcs_id, v.target_id,
           v.position_x, v.position_y, v.position_z);
}
template<class IO> void visit(IO& io, PipeResponse& v) {
    fields(io, v.magic, v.version, v.request_id, v.command, v.result, v.fcs_id,
           v.target_id, v.assignment, v.active, v.connected);
}
template<class T> auto write(T value) { Writer io; visit(io, value); return io.bytes; }
template<class T> bool read(std::span<const std::uint8_t> bytes, std::size_t size, T& value) {
    if (bytes.size() != size) return false; // Bounds checked once, before any field access.
    Reader io{bytes}; visit(io, value); return io.offset == size;
}
bool valid(const Payload& p) {
    if (!std::isfinite(p.rdr.position_x) || !std::isfinite(p.rdr.position_y) ||
        !std::isfinite(p.rdr.position_z) || !std::isfinite(p.rdr.azimuth) || !std::isfinite(p.rdr.elevation) ||
        !std::isfinite(p.rdr.field_of_view) || p.rdr.field_of_view < 0 || p.rdr.field_of_view > 360 ||
        !std::isfinite(p.fcs.position_x) || !std::isfinite(p.fcs.position_y) ||
        !std::isfinite(p.fcs.position_z) || !std::isfinite(p.fcs.azimuth) || !std::isfinite(p.fcs.elevation)) return false;
    if (p.rdr_count > 1 || p.fcs_count > 1 || p.target_count > target_capacity) return false;
    if (p.rdr.enabled > 1 || p.rdr.operating > 1 || p.rdr.connected > 1 ||
        p.fcs.enabled > 1 || p.fcs.operating > 1 || p.fcs.connected > 1 || p.fcs.laser_on > 1) return false;
    for (const auto& t : p.targets)
        if (t.valid > 1 || t.status > 5 || (t.status >= 4 && t.valid) || t.reserved || !std::isfinite(t.azimuth) ||
            !std::isfinite(t.elevation) || !std::isfinite(t.distance) ||
            t.distance < 0 || !std::isfinite(t.x) || !std::isfinite(t.y) || !std::isfinite(t.z)) return false;
    return true;
}
} // namespace
std::uint32_t checksum(std::span<const std::uint8_t> bytes) {
    std::uint32_t hash = 2166136261u;
    for (auto b : bytes) hash = (hash ^ b) * 16777619u;
    return hash;
}
std::vector<std::uint8_t> serialize(Frame value) {
    value.header.magic = frame_magic; value.header.version = frame_version;
    value.header.size = frame_size; value.header.payload_bytes = payload_size;
    value.header.checksum = checksum(write(value.payload));
    return write(value);
}
std::vector<std::uint8_t> serialize(PipeCommand value) { return write(value); }
std::vector<std::uint8_t> serialize(PipeResponse value) { return write(value); }
bool deserialize(std::span<const std::uint8_t> bytes, Frame& value) {
    Frame v;
    if (!read(bytes, frame_size, v) || v.header.magic != frame_magic ||
        v.header.version != frame_version || v.header.size != frame_size ||
        v.header.payload_bytes != payload_size ||
        v.header.checksum != checksum(bytes.subspan(header_size)) || !valid(v.payload)) return false;
    value = v; return true;
}
bool deserialize(std::span<const std::uint8_t> bytes, PipeCommand& value) {
    // Keep parsed fields for an InvalidRequest response with the original request ID.
    return read(bytes, command_size, value) && value.magic == pipe_magic && value.version == pipe_version;
}
bool deserialize(std::span<const std::uint8_t> bytes, PipeResponse& value) {
    PipeResponse v;
    if (!read(bytes, response_size, v) || v.magic != pipe_magic || v.version != pipe_version ||
        v.result > 23 || (v.command != 8 && v.assignment > 4) || v.active > 1 || v.connected > 1) return false;
    if (v.command == 8 && v.result == 0 &&
        (!std::isfinite(std::bit_cast<float>(v.fcs_id)) ||
         !std::isfinite(std::bit_cast<float>(v.target_id)) ||
         !std::isfinite(std::bit_cast<float>(v.assignment)))) return false;
    value = v; return true;
}
} // namespace c2::ipc
