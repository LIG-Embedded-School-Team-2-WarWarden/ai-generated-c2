#pragma once
#include "rdr_coordinates.hpp"
#include "nlohmann/json.hpp"

namespace c2::rdr {
// Read-only diagnostics: no retained positions and no writes to the control snapshot.
inline nlohmann::json live_diagnostics(const State& state, BasePoint origin, uint64_t timestamp_ms) {
    auto rows = nlohmann::json::array();
    const bool active = state.connected && state.reporting && state.mode == 2;
    for (const auto& point : correct_targets(state, origin)) {
        const auto& raw = state.targets.at(point.id);
        const bool valid = active && point.valid;
        nlohmann::json row{{"id", point.id}, {"raw_xyz", {raw.x, raw.y, raw.z}},
            {"observation_pan", raw.pan}, {"observation_tilt", raw.tilt},
            {"detected", raw.detected}, {"track_state", raw.track_state},
            {"world_valid", valid}, {"world_xyz", nullptr}};
        if (valid) row["world_xyz"] = {point.world.east, point.world.north, point.world.up};
        rows.push_back(std::move(row));
    }
    return {{"type", "rdr_live"}, {"timestamp_ms", timestamp_ms}, {"units", "m"},
        {"axes", {"east", "north", "up"}}, {"connected", state.connected},
        {"reporting", state.reporting}, {"mode", state.mode}, {"targets", rows}};
}
} // namespace c2::rdr
