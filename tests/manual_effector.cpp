#include "c2/server_options.hpp"
#include "c2/server_runtime.hpp"
#include "c2/dummy_assets.hpp"
#include "c2/protobuf_codec.hpp"
#include <iostream>
#include <limits>
#include <stdexcept>
#include <cmath>
void check(bool ok, const char* detail) { if (!ok) throw std::runtime_error(detail); }
int main() {
    std::uint64_t now = 10'000'000;
    auto config = c2::make_server_runtime_config(c2::ServerOptions{});
    std::vector<std::vector<std::byte>> sent;
    c2::ServerRuntime runtime(config, [&](auto bytes, const auto&) { sent.emplace_back(bytes.begin(), bytes.end()); });
    c2::Endpoint endpoint{"127.0.0.1", 6001};
    c2::DummyEffectorAsset device({
        {3, 1, now, c2::ComponentId::effector_asset, c2::ComponentId::command_and_control, 201, 123},
        c2::CoordinateFrame::project_frame, 0, 0, 0, 0});
    auto ingest = [&](auto message) {
        check(runtime.ingest(c2::protobuf::encode(c2::Envelope{message}), endpoint, now) == c2::InboundResult::accepted, "ingest rejected");
    };
    auto rejected = [&](float pan, float tilt) {
        return std::holds_alternative<c2::DispatchError>(runtime.point_effector_angles(201, pan, tilt, now));
    };
    check(rejected(30, 10), "unregistered command allowed");
    ingest(c2::AssetRegistration{
        {3, 1, now, c2::ComponentId::effector_asset, c2::ComponentId::command_and_control, 201, 123},
        c2::AssetRole::effector, 6001, c2::capability::effector_point,
        "manual-test", {}, {-90, 90, -45, 45}, false, 5000});
    ingest(device.heartbeat(now, 1));
    check(rejected(30, 10), "missing status allowed");
    ingest(device.status(now));
    check(rejected(91, 0) && rejected(0, 46), "limits not enforced");
    check(rejected(std::numeric_limits<float>::infinity(), 0), "nonfinite angle allowed");
    auto point = runtime.point_effector_angles(201, 30, 10, now);
    check(std::holds_alternative<c2::EffectorTurretCommand>(point), "point without target/pose rejected");
    check(rejected(20, 0), "overlapping point allowed");
    auto cmd = std::get<c2::EffectorTurretCommand>(point);
    auto response = device.handle(cmd, now);
    ingest(response.acknowledgement);
    ++now;
    ingest(device.status(now));
    auto assets = runtime.assets(now);
    check(assets[0].effector_status->current_pan_deg == 30 && assets[0].effector_status->current_tilt_deg == 10, "reported angles mismatch");
    check(runtime.command_outcomes().back().state == c2::CommandTerminalState::completed, "point not completed");
    auto stop = runtime.stop_effector(201, now);
    check(std::holds_alternative<c2::AttackCommand>(stop), "stop rejected");
    ingest(device.handle(std::get<c2::AttackCommand>(stop), now).acknowledgement);
    ++now;
    ingest(device.status(now));
    check(runtime.assets(now)[0].effector_status->state == c2::EffectorState::standby, "stop state mismatch");
    now += config.registry.status_timeout_us + 1;
    check(rejected(0, 0), "stale status allowed");
    check(runtime.tracks(now).empty() && runtime.assignments().empty(), "manual path created target state");
    ++now;
    ingest(device.status(now));
    c2::DummyObservationAsset observer({
        {3, 1, now, c2::ComponentId::observation_asset, c2::ComponentId::command_and_control, 101, 456},
        c2::CoordinateFrame::project_frame, 0, 0, 0, 0}, {-180, 180, -90, 90});
    ingest(c2::AssetRegistration{
        {3, 1, now, c2::ComponentId::observation_asset, c2::ComponentId::command_and_control, 101, 456},
        c2::AssetRole::observation, 5101, c2::capability::observation_scan,
        "aim-test", {}, {-180, 180, -90, 90}, false, 5000});
    ingest(observer.heartbeat(now, 1));
    auto aim_rejected = [&](std::uint64_t track, float x, float y, float z) {
        const auto before = sent.size();
        const auto r = runtime.point_observed_target(201, track, x, y, z, now);
        check(std::holds_alternative<c2::DispatchError>(r) && sent.size() == before, "unsafe aim transmitted");
    };
    aim_rejected(999, 0, 0, 0);
    ingest(observer.target(1, 2, 3, 4, 0.9F, now));
    aim_rejected(1, 2, 3, 4); // Coincident.
    aim_rejected(1, 2, 3, 0); // Undefined pan on vertical axis.
    aim_rejected(1, std::numeric_limits<float>::infinity(), 0, 0);
    for (int i = 0; i < 2; ++i) {
        const auto before = sent.size();
        auto aimed = runtime.point_observed_target(201, 1, 1, 2, 3, now);
        check(std::holds_alternative<c2::EffectorTurretCommand>(aimed), "valid aim rejected");
        const auto command = std::get<c2::EffectorTurretCommand>(aimed);
        check(std::abs(command.target_pan_deg - (i == 0 ? 45.0F : 26.56505F)) < 0.001F &&
              std::abs(command.target_tilt_deg - (i == 0 ? 35.26439F : 24.09484F)) < 0.001F,
              "relative atan2 solution or latest observation incorrect");
        check(sent.size() == before + 1 && runtime.assignments().empty(), "aim started tracking");
        ingest(device.handle(command, now).acknowledgement);
        ++now;
        ingest(device.status(now));
        if (i == 0) {
            // New coordinates must change the next manually requested angles.
            ingest(observer.target(1, 3, 3, 4, 0.9F, now));
        }
    }
    ingest(observer.target(2, -10, 0, 0, 0.9F, now));
    aim_rejected(2, 0, 0, 0); // Computed angle outside EFF limits.
    ingest(observer.target(3, 2, 3, 4, 0.9F, now + 100));
    aim_rejected(3, 1, 2, 3); // Future capture time must not be used.
    now += config.tracks.validity_us + 1;
    ingest(device.status(now));
    aim_rejected(1, 1, 2, 3);
    std::cout << "Manual effector passed: no OBS/pose/target, limits, busy guard, ACK, angles, STOP, stale status\n";
}
