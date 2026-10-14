#pragma once

#include <cstdint>
#include <optional>
#include <string_view>
#include <variant>

namespace c2 {
enum class ConsoleCommandKind {
    assets,
    targets,
    scan,
    observe,
    observation_stop,
    observation_home,
    assign,
    unassign,
    point,
    arm,
    start,
    stop,
    emergency_stop,
    emergency_stop_all,
    status,
    errors,
    outcomes,
    events,
    quit,
    development_pose,
    metrics,
    pending,
};

struct ConsoleCommand {
    ConsoleCommandKind kind{ConsoleCommandKind::status};
    std::uint64_t asset_id{};
    std::uint64_t track_id{};
    std::optional<std::uint64_t> effector_asset_id;
    float pan_deg{};
    float tilt_deg{};
    std::uint32_t duration_ms{};
    float x_m{};
    float y_m{};
    float z_m{};
    float azimuth_deg{};
};

enum class ConsoleParseError { empty, unknown_command, invalid_arguments };
using ConsoleParseResult = std::variant<ConsoleCommand, ConsoleParseError>;

[[nodiscard]] ConsoleParseResult parse_console_command(std::string_view line);
}  // namespace c2
