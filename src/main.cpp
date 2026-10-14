#include "c2/demo_adapters.hpp"
#include <chrono>
#include <iostream>
#include <sstream>
#include <charconv>

static bool identifier(std::istringstream& input, std::uint64_t& value) {
    std::string token;
    if (!(input >> token)) return false;
    auto [end, error] = std::from_chars(token.data(), token.data() + token.size(), value);
    return error == std::errc{} && end == token.data() + token.size() && value != 0;
}
static bool done(std::istringstream& input) { input >> std::ws; return input.eof(); }

static std::uint64_t now_us() {
    return std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
}
int main() {
    c2::DemoLidarProcessor lidar;
    c2::DemoPointingSolver solver;
    c2::ConsoleTransport transport(std::cout);
    c2::ControlCenter center(lidar, solver, transport);
    center.register_asset({201, {0, 0, 0}, 0, true, now_us()});
    std::cout << "AI C2 / SIMULATION ONLY / fixed demo angles\n"
              << "help | assets | register ASSET X Y Z AZIMUTH | heartbeat ASSET\n"
              << "targets | sample [OBSERVER] | assign ASSET TRACK | assignments\n"
              << "point ASSET TRACK | stop ASSET | stop-all | events | status | quit\n";
    for (std::string line; std::cout << "> " && std::getline(std::cin, line);) {
        std::istringstream input(line);
        std::string cmd; input >> cmd;
        std::uint64_t asset{}, track{};
        center.tick(now_us());
        if (cmd.empty()) continue;
        if (cmd == "quit" && done(input)) break;
        if (cmd == "assets" && done(input)) {
            for (auto& [id, a] : center.assets()) std::cout << id << " connected=" << a.connected
                << " xyz=" << a.project_position.x << ',' << a.project_position.y << ',' << a.project_position.z
                << " azimuth=" << a.azimuth_deg << '\n';
        } else if (cmd == "register" && identifier(input, asset)) {
            c2::Position p; double azimuth{};
            if (!(input >> p.x >> p.y >> p.z >> azimuth) || !done(input)) {
                std::cout << "usage: register ASSET X Y Z AZIMUTH\n"; continue;
            }
            try {
                center.register_asset({asset, p, azimuth, true, now_us()});
                std::cout << center.last_result() << '\n';
            } catch (const std::exception& e) { std::cout << e.what() << '\n'; }
        } else if (cmd == "heartbeat" && identifier(input, asset) && done(input)) {
            const auto now = now_us(); center.heartbeat(asset, now, now);
            std::cout << center.last_result() << '\n';
        } else if (cmd == "targets" && done(input)) {
            for (auto& [id, t] : center.tracks()) std::cout << id << " observer=" << t.observer_id
                << " xyz=" << t.project_position.x << ',' << t.project_position.y << ',' << t.project_position.z
                << " age_us=" << now_us() - t.measured_us << '\n';
        } else if (cmd == "sample") {
            asset = 101;
            if (!done(input) && (!identifier(input, asset) || !done(input))) {
                std::cout << "usage: sample [OBSERVER]\n"; continue;
            }
            auto now = now_us(); center.ingest({asset, now, {{10, 20, 2}}}, now);
            std::cout << "sample ingested; validity 2 seconds\n";
        } else if ((cmd == "point" || cmd == "assign") && identifier(input, asset) &&
                   identifier(input, track) && done(input)) {
            if (cmd == "point") center.point(asset, track, now_us());
            else center.assign(asset, track, now_us());
            std::cout << center.last_result() << '\n';
        } else if (cmd == "stop" && identifier(input, asset) && done(input)) {
            center.stop(asset); std::cout << center.last_result() << '\n';
        } else if (cmd == "stop-all" && done(input)) {
            for (auto& [id, a] : center.assets()) {
                center.stop(id); std::cout << id << ' ' << center.last_result() << '\n';
            }
        } else if (cmd == "assignments" && done(input)) {
            for (auto& [id, target] : center.assignments()) std::cout << "asset=" << id << " track=" << target << '\n';
        } else if (cmd == "events" && done(input)) {
            for (auto& e : center.events()) std::cout << e.sequence << ' ' << e.action
                << " asset=" << e.asset_id << " track=" << e.track_id << ' ' << e.result << '\n';
        } else if (cmd == "status" && done(input)) {
            std::cout << "assets=" << center.assets().size() << " tracks=" << center.tracks().size()
                << " assignments=" << center.assignments().size() << " events=" << center.events().size() << '\n';
        } else if (cmd == "help" && done(input)) {
            std::cout << "register ASSET X Y Z AZIMUTH | heartbeat ASSET | sample [OBSERVER]\n"
                      << "assets | targets | assign ASSET TRACK | assignments | point ASSET TRACK\n"
                      << "stop ASSET | stop-all | events | status | quit\n";
        } else std::cout << "unknown command or invalid arguments\n";
    }
}
