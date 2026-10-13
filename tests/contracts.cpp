#include "c2/demo_adapters.hpp"
#include <iostream>
#include <limits>
#include <stdexcept>

void check(bool value) { if (!value) throw std::runtime_error("contract failed"); }
struct Transport : c2::IAssetTransport {
    int points{}, stops{}; bool accept{true};
    bool send_point(const c2::PointCommand&) override { ++points; return accept; }
    bool send_stop(std::uint64_t) override { ++stops; return accept; }
};
int main() {
    c2::DemoLidarProcessor lidar; c2::DemoPointingSolver solver; Transport transport;
    c2::ControlCenter c(lidar, solver, transport);
    c.register_asset({201, {}, 0, true});
    c.ingest({101, 100, {{1, 2, 3}}}, 100);
    c.ingest({102, 100, {{4, 5, 6}}}, 100);
    check(c.tracks().size() == 2); // Same local detection ID, different observers.
    check(c.point(201, 1, 101));
    check(!c.point(201, 1, 2'000'101));
    check(!c.point(999, 1, 101));
    c.ingest({101, 99, {{9, 9, 9}}}, 100);
    check(c.tracks().at(1).project_position.x == 1); // Reject reordered reports.
    c.ingest({103, 101, {{1, 2, 3}}}, 100);
    c.ingest({103, 100, {{std::numeric_limits<double>::quiet_NaN(), 2, 3}}}, 100);
    check(c.tracks().size() == 2);
    transport.accept = false;
    check(!c.point(201, 1, 101)); // Queue failure propagates.
    c.register_asset({201, {}, 0, false});
    check(!c.point(201, 1, 101));
    transport.accept = true;
    check(c.stop(201)); // Stop is allowed for a known disconnected asset.
    check(transport.points == 2 && transport.stops == 1);
    std::cout << "contracts passed\n";
}
