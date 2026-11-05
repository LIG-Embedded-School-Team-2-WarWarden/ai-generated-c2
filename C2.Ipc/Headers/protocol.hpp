#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <type_traits>
#include <vector>

// GUI frame v3 remains compatible with the unmodified external GUI.
// The independent command/response pipe remains v3.
// These are domain/ABI structs. Wire bytes are written field by field in the codec;
// no memcpy/reinterpret_cast of a struct is used for serialization.
namespace c2::ipc {
constexpr std::uint32_t frame_magic = 0x314d4354, frame_version = 3;
constexpr std::uint32_t pipe_magic = 0x50495045, pipe_version = 3;
constexpr std::size_t header_size = 32, payload_size = 460, frame_size = 492;
constexpr std::size_t command_size = 40, response_size = 40, target_capacity = 10;
constexpr wchar_t mapping_name[] = L"Local\\ControlStationMock.SharedMemory.v3";
constexpr wchar_t mutex_name[] = L"Local\\ControlStationMock.SharedMemory.Mutex.v3";
constexpr wchar_t pipe_path[] = L"\\\\.\\pipe\\ECS_NamedPipe";

enum class Command : std::uint32_t {
    status, assign, cancel, fire, set_rdr_elevation,
    set_asset_position, set_rdr_operating
};
enum class Result : std::uint32_t {
    success, invalid_request, asset_unavailable, not_assigned,
    already_assigned, not_supported, device_error,
    status_pending, target_pending, target_changed, device_fault, alignment_required,
    device_busy, response_timeout, device_rejected
};
enum class Assignment : std::uint32_t { idle, adjusting, assigned, failed, waiting };

#pragma pack(push, 4)
struct RdrAsset {
    std::uint32_t id{}, enabled{}, operating{}, connected{};
    float azimuth{}, elevation{};
    float position_x{}, position_y{}, position_z{}, field_of_view{};
};
struct FcsAsset {
    std::uint32_t id{}, enabled{}, operating{}, connected{}, laser_on{};
    float azimuth{}, elevation{};
    float position_x{}, position_y{}, position_z{};
};
struct Target {
    std::uint32_t id{};
    float azimuth{}, elevation{};
    float distance{}, x{}, y{}, z{};
    std::uint32_t rdr_id{};
    std::uint8_t status{}, valid{};
    std::uint16_t reserved{};
};
struct Payload {
    RdrAsset rdr{};
    FcsAsset fcs{};
    std::uint32_t rdr_count{}, fcs_count{}, selected_rdr{}, selected_fcs{}, target_count{};
    std::array<Target, target_capacity> targets{};
};
struct FrameHeader {
    std::uint32_t magic{frame_magic}, version{frame_version}, size{frame_size}, sequence{};
    std::uint64_t timestamp_ms{};
    std::uint32_t payload_bytes{payload_size}, checksum{};
};
struct Frame { FrameHeader header{}; Payload payload{}; };
struct PipeCommand {
    std::uint32_t magic{pipe_magic}, version{pipe_version}, command{}, request_id{};
    std::uint32_t rdr_id{}, fcs_id{}, target_id{};
    float position_x{}, position_y{}, position_z{};
};
struct PipeResponse {
    std::uint32_t magic{pipe_magic}, version{pipe_version}, request_id{}, command{}, result{};
    std::uint32_t fcs_id{}, target_id{}, assignment{}, fire{}, connected{};
};
#pragma pack(pop)
static_assert(sizeof(RdrAsset) == 40 && sizeof(FcsAsset) == 40 && sizeof(Target) == 36);
static_assert(sizeof(Payload) == payload_size && sizeof(FrameHeader) == header_size);
static_assert(sizeof(Frame) == frame_size && sizeof(PipeCommand) == command_size);
static_assert(sizeof(PipeResponse) == response_size);
static_assert(offsetof(FrameHeader, timestamp_ms) == 16 && offsetof(Payload, targets) == 100);
static_assert(offsetof(PipeCommand, position_x) == 28);

std::uint32_t checksum(std::span<const std::uint8_t> bytes);
std::vector<std::uint8_t> serialize(Frame value);
std::vector<std::uint8_t> serialize(PipeCommand value);
std::vector<std::uint8_t> serialize(PipeResponse value);
bool deserialize(std::span<const std::uint8_t>, Frame&);
bool deserialize(std::span<const std::uint8_t>, PipeCommand&);
bool deserialize(std::span<const std::uint8_t>, PipeResponse&);
} // namespace c2::ipc
