#include "c2/demo_adapters.hpp"
#include <iostream>
#include <limits>
#include <stdexcept>
void check(bool v, const char* msg) { if (!v) throw std::runtime_error(msg); }
struct Transport : c2::IAssetTransport {
    int points{}, stops{}; bool accept{true};
    bool send_point(const c2::PointCommand&) override { ++points; return accept; }
    bool send_stop(std::uint64_t) override { ++stops; return accept; }
};
struct RejectingSolver : c2::IPointingSolver {
    std::optional<c2::Angles> solve(const c2::Asset&, const c2::Observation&) const override { return std::nullopt; }
};
struct WrongSource : c2::ILidarProcessor {
    std::vector<c2::Observation> process(const c2::LidarFrame& f) override { return {{999, 1, {}, f.measured_us}}; }
};
int main() {
    c2::DemoLidarProcessor lidar; c2::DemoPointingSolver solver; Transport transport;
    c2::ControlCenter c(lidar, solver, transport);
    c.register_asset({201, {}, 0, true, 100});
    c.register_asset({202, {}, 0, true, 100});
    c.ingest({101, 100, {{1, 2, 3}}}, 100);
    c.ingest({102, 100, {{4, 5, 6}}}, 100);
    check(c.tracks().size() == 2, "observer identity collision");
    check(c.assign(201, 1, 101), "initial assignment failed");
    check(!c.assign(202, 1, 101), "track assigned twice");
    check(!c.assign(201, 2, 101), "busy asset reassigned");
    check(!c.point(201, 2, 101), "manual point bypassed assignment");
    check(!c.point(999, 1, 101), "unknown asset accepted");
    c.ingest({101, 99, {{9, 9, 9}}}, 101);
    check(c.tracks().at(1).project_position.x == 1, "reordered report replaced track");
    c.ingest({103, 102, {{1, 2, 3}}}, 101);
    c.ingest({103, 100, {{std::numeric_limits<double>::quiet_NaN(), 2, 3}}}, 101);
    check(c.tracks().size() == 2, "invalid report admitted");
    transport.accept = false;
    check(!c.assign(202, 2, 101), "queue failure ignored");
    check(!c.assignments().contains(202), "failed queue reserved asset");
    check(!c.stop(201), "failed stop reported success");
    check(!c.assignments().contains(201), "failed stop retained assignment");
    transport.accept = true;
    check(c.assign(201, 1, 101), "reassignment failed");
    c.tick(2'000'100);
    check(c.tracks().size() == 2, "boundary expiry too early");
    c.tick(2'000'101);
    check(c.tracks().empty() && c.assignments().empty(), "expiry retained assignment");
    check(!c.point(201, 1, 2'000'101), "expired point accepted");
    c.ingest({101, 2'000'102, {{1, 2, 3}}}, 2'000'102);
    check(c.tracks().begin()->first == 3, "expired track ID reused");
    c.tick(3'000'101);
    check(!c.assets().at(201).connected, "heartbeat timeout ignored");
    check(!c.heartbeat(201, 99, 3'000'101), "reordered heartbeat accepted");
    check(!c.heartbeat(201, 4'000'000, 3'000'101), "future heartbeat accepted");
    check(c.heartbeat(201, 3'000'101, 3'000'101), "heartbeat did not reconnect");
    check(c.assign(201, 3, 3'000'101), "reconnected assignment failed");
    c.register_asset({201, {}, 0, false, 3'000'101});
    check(c.assignments().empty(), "registration retained assignment");
    check(c.stop(201), "known offline asset cannot stop");
    c2::ControlCenter bounded(lidar, solver, transport, {1, 1, 2, 100, 100});
    bounded.register_asset({1, {}, 0, true, 1});
    bool capacity = false;
    try { bounded.register_asset({2, {}, 0, true, 1}); } catch (const std::length_error&) { capacity = true; }
    check(capacity, "asset cap ignored");
    bounded.ingest({10, 1, {{1, 1, 1}, {2, 2, 2}}}, 1);
    check(bounded.tracks().size() == 1, "track cap ignored");
    bounded.ingest({10, 2, {{3, 3, 3}}}, 2);
    check(bounded.tracks().at(1).project_position.x == 3, "existing track blocked by capacity");
    check(bounded.events().size() == 2, "history cap ignored");
    bounded.tick(103);
    bounded.ingest({10, 104, {{1, 1, 1}}}, 104);
    check(bounded.tracks().size() == 1, "expiry did not reclaim capacity");
    RejectingSolver rejecting;
    c2::ControlCenter no_solution(lidar, rejecting, transport);
    no_solution.register_asset({1, {}, 0, true, 100});
    no_solution.ingest({10, 100, {{1, 1, 1}}}, 100);
    const int previous_points = transport.points;
    check(!no_solution.assign(1, 1, 100), "solver rejection ignored");
    check(transport.points == previous_points, "failed solve sent command");
    WrongSource wrong;
    c2::ControlCenter bad_source(wrong, solver, transport);
    bad_source.ingest({10, 100, {{1, 1, 1}}}, 100);
    check(bad_source.tracks().empty(), "processor changed observer identity");
    bool invalid_limits = false;
    try { c2::ControlCenter invalid(lidar, solver, transport, {0, 1, 1, 1, 1}); }
    catch (const std::invalid_argument&) { invalid_limits = true; }
    check(invalid_limits, "invalid limits accepted");
    std::cout << "contracts passed: identity, expiry, assignment, heartbeat, limits, adapters\n";
}
