#include "c2/server_console.hpp"

#include <cmath>
#include <sstream>
#include <string>

namespace c2 {
namespace {
bool finished(std::istringstream& input) {
    input >> std::ws;
    return input.eof();
}

bool positive(const std::uint64_t value) noexcept { return value != 0; }
}

ConsoleParseResult parse_console_command(const std::string_view line) {
    std::istringstream input(std::string{line});
    std::string name;
    if (!(input >> name)) return ConsoleParseError::empty;
    ConsoleCommand command;
    if (name == "assets") command.kind = ConsoleCommandKind::assets;
    else if (name == "targets") command.kind = ConsoleCommandKind::targets;
    else if (name == "pending") command.kind = ConsoleCommandKind::pending;
    else if (name == "metrics") command.kind = ConsoleCommandKind::metrics;
    else if (name == "status") command.kind = ConsoleCommandKind::status;
    else if (name == "errors") command.kind = ConsoleCommandKind::errors;
    else if (name == "outcomes") command.kind = ConsoleCommandKind::outcomes;
    else if (name == "events") command.kind = ConsoleCommandKind::events;
    else if (name == "quit") command.kind = ConsoleCommandKind::quit;
    else if (name == "estop-all") command.kind = ConsoleCommandKind::emergency_stop_all;
    else if (name == "dev-pose") {
        command.kind = ConsoleCommandKind::development_pose;
        if (!(input >> command.asset_id >> command.x_m >> command.y_m >>
              command.z_m >> command.azimuth_deg) || !positive(command.asset_id) ||
            !std::isfinite(command.x_m) || !std::isfinite(command.y_m) ||
            !std::isfinite(command.z_m) || !std::isfinite(command.azimuth_deg) ||
            command.azimuth_deg < 0 || command.azimuth_deg >= 360)
            return ConsoleParseError::invalid_arguments;
    }
    else if (name == "scan" || name == "observe") {
        command.kind = name == "scan" ? ConsoleCommandKind::scan
                                       : ConsoleCommandKind::observe;
        const auto arguments = input.tellg();
        bool legacy{};
        if (!(input >> command.asset_id >> command.pan_deg >> command.tilt_deg)) {
            input.clear();
            input.seekg(arguments);
            command.asset_id = 0;
            legacy = true;
            if (!(input >> command.pan_deg >> command.tilt_deg))
                return ConsoleParseError::invalid_arguments;
        }
        if ((!legacy && !positive(command.asset_id)) ||
            !std::isfinite(command.pan_deg) ||
            !std::isfinite(command.tilt_deg))
            return ConsoleParseError::invalid_arguments;
    } else if (name == "obs-stop" || name == "obs-home") {
        command.kind = name == "obs-stop"
            ? ConsoleCommandKind::observation_stop
            : ConsoleCommandKind::observation_home;
        if (input >> command.asset_id) {
            if (!positive(command.asset_id))
                return ConsoleParseError::invalid_arguments;
        } else {
            input.clear();
        }
    } else if (name == "assign") {
        command.kind = ConsoleCommandKind::assign;
        if (!(input >> command.track_id) || !positive(command.track_id))
            return ConsoleParseError::invalid_arguments;
        std::uint64_t asset_id{};
        if (input >> asset_id) {
            if (!positive(asset_id)) return ConsoleParseError::invalid_arguments;
            command.effector_asset_id = asset_id;
        } else {
            input.clear();
        }
    } else if (name == "unassign" || name == "point" || name == "arm") {
        command.kind = name == "unassign" ? ConsoleCommandKind::unassign
                     : name == "point" ? ConsoleCommandKind::point
                                        : ConsoleCommandKind::arm;
        if (!(input >> command.track_id) || !positive(command.track_id))
            return ConsoleParseError::invalid_arguments;
    } else if (name == "start") {
        command.kind = ConsoleCommandKind::start;
        if (!(input >> command.track_id >> command.duration_ms) ||
            !positive(command.track_id) || command.duration_ms == 0)
            return ConsoleParseError::invalid_arguments;
    } else if (name == "stop" || name == "estop") {
        command.kind = name == "stop" ? ConsoleCommandKind::stop
                                       : ConsoleCommandKind::emergency_stop;
        if (input >> command.asset_id) {
            if (!positive(command.asset_id))
                return ConsoleParseError::invalid_arguments;
        } else {
            input.clear();
        }
    } else {
        return ConsoleParseError::unknown_command;
    }
    return finished(input) ? ConsoleParseResult{command}
                           : ConsoleParseResult{ConsoleParseError::invalid_arguments};
}
}  // namespace c2
