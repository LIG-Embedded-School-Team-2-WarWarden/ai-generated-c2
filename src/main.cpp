#include "c2/demo_adapters.hpp"
#include <chrono>
#include <iostream>
#include <sstream>

static std::uint64_t now_us() {
    return std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
}
int main() {
    c2::DemoLidarProcessor lidar;
    c2::DemoPointingSolver solver;
    c2::ConsoleTransport transport(std::cout);
    c2::ControlCenter center(lidar, solver, transport);
    center.register_asset({201, {0, 0, 0}, 0, true});
    std::cout << "AI C2 / SIMULATION ONLY / fixed demo angles\n"
              << "assets | targets | sample | point ASSET TRACK | stop ASSET | quit\n";
    for (std::string line; std::cout << "> " && std::getline(std::cin, line);) {
        std::istringstream input(line);
        std::string cmd; input >> cmd;
        std::uint64_t asset{}, track{};
        if (cmd == "quit") break;
        if (cmd == "assets") {
            for (auto& [id, a] : center.assets()) std::cout << id << " connected=" << a.connected << '\n';
        } else if (cmd == "targets") {
            for (auto& [id, t] : center.tracks()) std::cout << id << " observer=" << t.observer_id
                << " xyz=" << t.project_position.x << ',' << t.project_position.y << ',' << t.project_position.z
                << " age_us=" << now_us() - t.measured_us << '\n';
        } else if (cmd == "sample") {
            auto now = now_us(); center.ingest({101, now, {{10, 20, 2}}}, now);
            std::cout << "sample ingested; validity 2 seconds\n";
        } else if (cmd == "point" && (input >> asset >> track)) {
            std::cout << (center.point(asset, track, now_us()) ? "queued\n" : "rejected\n");
        } else if (cmd == "stop" && (input >> asset)) {
            std::cout << (center.stop(asset) ? "queued\n" : "rejected\n");
        } else std::cout << "unknown command or invalid arguments\n";
    }
}
