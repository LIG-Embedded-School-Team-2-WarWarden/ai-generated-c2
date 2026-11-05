#include "rdr_live_diagnostics.hpp"
#include <stdexcept>

static void check(bool value, const char* message) {
    if (!value) throw std::runtime_error(message);
}
int rdr_live_diagnostics_entry() {
    c2::rdr::State state;
    state.connected = state.reporting = true;
    state.mode = 2;
    auto& raw = state.targets[7];
    raw.id = 7; raw.status = 0; raw.track_state = raw.detected = 1;
    raw.x = -2; raw.y = 1; raw.z = 3; raw.pan = 1800; raw.tilt = 0;
    const c2::rdr::BasePoint origin{10, 20, 30};
    auto read = [&] { return c2::rdr::live_diagnostics(state, origin, 1234); };
    const auto first = read();
    check(first["targets"][0]["world_xyz"] == nlohmann::json({12, 23, 31}),
        "diagnostic world XYZ includes installation offset");
    check(first["timestamp_ms"] == 1234 && first["type"] == "rdr_live", "diagnostic metadata");
    raw.x = -5; raw.y = 2; raw.z = 8;
    check(read()["targets"][0]["world_xyz"] == nlohmann::json({15, 28, 32}),
        "same ID diagnostic reflects latest observation without reacquisition");
    check(first["targets"][0]["world_xyz"] == nlohmann::json({12, 23, 31}),
        "previous diagnostic result remains an independent snapshot");
    raw.pan = 2700;
    auto point = read()["targets"][0]["world_xyz"];
    check(std::abs(point[0].get<double>() - 18) < 1e-9 &&
          std::abs(point[1].get<double>() - 15) < 1e-9,
        "diagnostic uses latest observation pose");
    raw.pan = -32768;
    check(read()["targets"][0]["world_xyz"].is_null(), "unknown pose has no diagnostic world position");
    raw.pan = 1800; raw.status = 2; raw.track_state = 0;
    check(!read()["targets"][0]["world_valid"].get<bool>(), "lost data is invalid");
    raw.status = 0; raw.track_state = 1;
    state.connected = false;
    check(read()["targets"][0]["world_xyz"].is_null(), "disconnected data is not presented as live");
    state.connected = true; state.reporting = false;
    check(read()["targets"][0]["world_xyz"].is_null(), "reporting off invalidates diagnostic output");
    state.reporting = true; state.mode = 0;
    check(read()["targets"][0]["world_xyz"].is_null(), "stopped sensor invalidates diagnostic output");
    check(raw.x == -5 && raw.y == 2 && raw.z == 8, "diagnostic does not modify sensor state");
    return 0;
}
