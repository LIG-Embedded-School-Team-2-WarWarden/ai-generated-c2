#include "c2mvp/domain.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>
using namespace c2mvp;
namespace {
void check(bool value, const char *why) {
    if (!value)
        throw std::runtime_error(why);
}
struct FakeClock : Clock {
    double wall = 1700000000, mono = 100;
    double time() const override {
        return wall;
    }
    double monotonic() const override {
        return mono;
    }
    void advance(double s) {
        wall += s;
        mono += s;
    }
};
struct FakeTransport : DatagramTransport {
    std::vector<std::string> packets;
    void send(const std::string &data, const Endpoint &) override {
        packets.push_back(data);
    }
    std::optional<Datagram> receive() override {
        return {};
    }
    Endpoint address() const override {
        return {"127.0.0.1", 5000};
    }
    pb::Envelope last(pb::Envelope::BodyCase kind) const {
        for (auto it = packets.rbegin(); it != packets.rend(); ++it) {
            pb::Envelope e;
            e.ParseFromString(*it);
            if (e.body_case() == kind)
                return e;
        }
        throw std::runtime_error("expected packet missing");
    }
    size_t count(pb::Envelope::BodyCase kind) const {
        size_t n = 0;
        for (auto &b : packets) {
            pb::Envelope e;
            e.ParseFromString(b);
            if (e.body_case() == kind)
                ++n;
        }
        return n;
    }
};
struct FakeDetector : PointDetector {
    int calls = 0;
    std::vector<Object> detect(unsigned, unsigned, const std::vector<pb::Point> &p) override {
        ++calls;
        return {{"object-1", .2, .1, 3, static_cast<unsigned>(p.size())}};
    }
};
struct Solver : PointingSolver {
    double pan = 13, tilt = 7;
    std::pair<double, double> solve(const Object &, const Installation &,
                                    const Installation &) const override {
        return {pan, tilt};
    }
};
Config config() {
    Config c;
    c.simulation = true;
    c.observer = "101";
    c.uncertainty = 10;
    Installation obs;
    obs.role = "OBS";
    obs.revision = 1;
    obs.calibration = "synthetic-obs-zero-v1";
    obs.profile = "synthetic-organized-v1";
    obs.rotation = {0, 0, 1, 1, 0, 0, 0, 1, 0};
    Installation eff;
    eff.role = "EFF";
    eff.revision = 1;
    eff.calibration = "synthetic-eff-zero-v1";
    eff.position = {0, .1, 0};
    eff.rotation = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    c.devices = {{"101", obs}, {"201", eff}};
    return c;
}
struct Fixture {
    FakeClock clock;
    FakeTransport transport;
    FakeDetector detector;
    Solver solver;
    Engine engine;
    Fixture() : engine(config(), clock, transport, detector, solver, false) {
        for (auto id : {"101", "201"}) {
            Peer p;
            p.id = id;
            p.session = uid();
            p.boot = uid();
            p.address = {"127.0.0.1", 5001};
            p.installed = p.install_result = p.install_sent = p.mode = p.mode_sent = true;
            p.obs_session = uid();
            p.collection = uid();
            p.collecting = p.id == "101";
            p.mode_applied = clock.time() - 1;
            pb::CapabilityReport c;
            c.set_device_id(id);
            c.set_absolute_pointing(true);
            c.set_measured_angle_feedback(true);
            c.set_stop_supported(true);
            c.set_drain_supported(true);
            c.set_point_completion_timeout_ms(1000);
            c.set_stop_completion_timeout_ms(1000);
            c.set_pan_min_deg(-90);
            c.set_pan_max_deg(90);
            c.set_tilt_min_deg(-45);
            c.set_tilt_max_deg(45);
            p.cap = c;
            engine.devices.peers[id] = p;
            fresh(id);
        }
        observed(1);
    }
    Peer &peer(const std::string &id = "201") {
        return engine.devices.peers.at(id);
    }
    void fresh(const std::string &id = "201",
               pb::MeasurementState measurement = pb::MEASUREMENT_STATE_COLLECTING) {
        auto &p = peer(id);
        pb::DeviceStatus s;
        s.set_device_id(id);
        s.set_reported_at(clock.utc());
        s.set_device_state(pb::DEVICE_STATE_IDLE);
        s.set_motion_state(pb::MOTION_STATE_STOPPED);
        s.set_measurement_state(measurement);
        s.set_active_installation_revision(1);
        s.set_clock_state(pb::CLOCK_STATE_SYNCED);
        s.set_clock_uncertainty_ms(1);
        s.set_estop_latch_state(pb::EMERGENCY_LATCH_STATE_RELEASED);
        auto *a = s.mutable_angles();
        a->set_sample_sequence(p.sample + 1);
        a->set_pan_valid(true);
        a->set_tilt_valid(true);
        a->set_pan_deg(0);
        a->set_tilt_deg(0);
        a->set_pan_measured_at(clock.utc());
        a->set_tilt_measured_at(clock.utc());
        engine.devices.status(p, s);
    }
    void observed(uint64_t frame) {
        Observation r;
        r.meta.set_frame_id(frame);
        r.meta.set_captured_start_at(clock.utc());
        r.meta.set_collection_session_id(peer("101").collection);
        r.meta.set_installation_revision(1);
        r.objects = {{"object-1", .2, .1, 3, 48}};
        r.verified = true;
        r.received = clock.monotonic();
        engine.observation.cloud.state = "CURRENT";
        engine.observation.cloud.result = r;
    }
    pb::Envelope reply(const pb::Envelope &q,
                       pb::RequestResult result = pb::REQUEST_RESULT_COMPLETED) {
        pb::Envelope e;
        pb::Result *r = nullptr;
        if (q.has_control_open()) {
            auto &b = q.control_open();
            auto *out = e.mutable_control_result();
            out->set_active_control_session_id(b.control_session_id());
            out->set_control_epoch(b.control_epoch());
            r = out->mutable_outcome();
        }
        if (q.has_laser_command()) {
            const auto &b = q.laser_command();
            auto *out = e.mutable_laser_result();
            out->set_control_session_id(b.control_session_id());
            out->set_control_epoch(b.control_epoch());
            out->set_command_sequence(b.command_sequence());
            out->set_actual_laser_state(b.operation() == pb::LaserCommand::OPERATION_ON
                                           ? pb::LASER_STATE_ON : pb::LASER_STATE_OFF);
            out->set_state_observed_at(clock.utc());
            r = out->mutable_outcome();
        }
        if (q.has_pointing_command()) {
            auto &b = q.pointing_command();
            auto *out = e.mutable_pointing_result();
            out->set_control_session_id(b.control_session_id());
            out->set_control_epoch(b.control_epoch());
            out->set_command_sequence(b.command_sequence());
            out->set_actual_motion_state(pb::MOTION_STATE_STOPPED);
            r = out->mutable_outcome();
        }
        if (q.has_measurement_command()) {
            auto &b = q.measurement_command();
            auto *out = e.mutable_measurement_result();
            out->set_collection_session_id(b.collection_session_id());
            out->set_actual_state(b.operation() == pb::MeasurementCommand::OPERATION_START
                                      ? pb::MEASUREMENT_STATE_COLLECTING
                                      : pb::MEASUREMENT_STATE_STOPPED);
            r = out->mutable_outcome();
        }
        if (q.has_obs_mode_command()) {
            auto &b = q.obs_mode_command();
            auto *out = e.mutable_obs_mode_result();
            out->set_obs_id("101");
            out->set_observation_mode_revision(b.observation_mode_revision());
            out->set_mode(pb::OBSERVATION_MODE_FIXED_VIEW);
            out->set_actual_state(pb::MOTION_STATE_STOPPED);
            out->set_mode_applied_at(clock.utc());
            r = out->mutable_outcome();
        }
        if (q.has_obs_motion_command()) {
            auto &b = q.obs_motion_command();
            auto *out = e.mutable_obs_motion_result();
            out->set_obs_id("101");
            out->set_obs_control_session_id(b.obs_control_session_id());
            out->set_control_epoch(b.control_epoch());
            out->set_command_sequence(b.command_sequence());
            out->set_actual_motion_state(pb::MOTION_STATE_STOPPED);
            r = out->mutable_outcome();
        }
        check(r != nullptr, "unsupported fixture command");
        r->set_request_id(request_id(q));
        r->set_result(result);
        r->set_reported_at(clock.utc());
        if (result != pb::REQUEST_RESULT_ACCEPTED)
            r->set_completed_at(clock.utc());
        return e;
    }
    void result(const pb::Envelope &q, pb::RequestResult result = pb::REQUEST_RESULT_COMPLETED) {
        engine.requests.result(peer(q.has_obs_motion_command() || q.has_obs_mode_command() ||
                                            q.has_measurement_command()
                                        ? "101"
                                        : "201"),
                               reply(q, result));
    }
    pb::Envelope point() {
        pb::Envelope e;
        auto *q = e.mutable_pointing_command();
        q->set_request_id(uid());
        q->set_control_session_id(uid());
        q->set_control_epoch(1);
        q->set_command_sequence(7);
        q->set_operation(pb::PointingCommand::OPERATION_POINT);
        return e;
    }
    void aim() {
        engine.effector.begin("201");
        result(transport.last(pb::Envelope::kControlOpen));
        engine.effector.tick(clock.monotonic());
    }
    void obs_moving() {
        engine.observer.point(10, 5);
        engine.observer.tick(clock.monotonic());
        result(transport.last(pb::Envelope::kMeasurementCommand));
        fresh("101", pb::MEASUREMENT_STATE_STOPPED);
        engine.observer.tick(clock.monotonic());
        result(transport.last(pb::Envelope::kObsModeCommand));
    }
};
int passed = 0;
void test(const char *name, const std::function<void()> &fn) {
    fn();
    ++passed;
    std::cout << "PASS " << name << '\n';
}
} // namespace
int main() {
    try {
        test("laser requires both capability advertisements", [] {
            Fixture f;
            f.aim();
            f.peer().cap->set_laser_control_supported(true);
            bool rejected = false;
            try { f.engine.command("laser on 201 1000"); }
            catch (const std::invalid_argument &) { rejected = true; }
            check(rejected && f.transport.count(pb::Envelope::kLaserCommand) == 0,
                  "laser sent to unadvertised peer");
            f.peer().laser_supported = true;
            f.peer().cap->set_laser_control_supported(false);
            rejected = false;
            try { f.engine.command("laser on 201 1000"); }
            catch (const std::invalid_argument &) { rejected = true; }
            check(rejected, "missing capability report accepted");
        });
        test("laser validates duration and correlates session epoch sequence", [] {
            Fixture f; f.aim();
            f.peer().laser_supported = true;
            f.peer().cap->set_laser_control_supported(true);
            for (auto duration : {"0", "5001", "1.5", "nan"}) {
                bool rejected = false;
                try { f.engine.command(std::string("laser on 201 ") + duration); }
                catch (const std::exception &) { rejected = true; }
                check(rejected, "invalid laser duration accepted");
            }
            f.engine.command("laser on 201 1000");
            auto q = f.transport.last(pb::Envelope::kLaserCommand);
            check(q.laser_command().command_sequence() == 1 &&
                      q.laser_command().duration_ms() == 1000, "laser sequence not independent");
            auto e = f.reply(q);
            e.mutable_laser_result()->set_control_epoch(99);
            f.engine.requests.result(f.peer(), e);
            check(f.engine.requests.has("201", pb::Envelope::kLaserResult), "foreign epoch accepted");
            f.result(q);
            check(!f.engine.requests.has("201", pb::Envelope::kLaserResult), "laser result not handled");
            check(f.engine.snapshot()["lasers"]["201"]["state"] == "LASER_STATE_UNKNOWN",
                  "result inferred telemetry state");
        });
        test("laser telemetry expires and cannot regress with UDP reordering", [] {
            Fixture f; f.aim();
            f.peer().laser_supported = true;
            f.peer().cap->set_laser_control_supported(true);
            f.engine.command("laser on 201 1000");
            auto c = f.engine.effector.sessions.at("201");
            pb::LaserStatus s;
            s.set_device_id("201"); s.set_control_session_id(c->session);
            s.set_control_epoch(c->epoch); s.set_last_accepted_sequence(1);
            s.set_reported_at(f.clock.utc()); s.set_state_observed_at(f.clock.utc());
            s.set_laser_state(pb::LASER_STATE_ON); s.set_remaining_duration_ms(1000);
            f.engine.effector.laser_status(f.peer(), s);
            check(f.engine.snapshot()["lasers"]["201"]["state"] == "LASER_STATE_ON", "status ignored");
            s.set_laser_state(pb::LASER_STATE_OFF);
            f.engine.effector.laser_status(f.peer(), s);
            check(f.engine.snapshot()["lasers"]["201"]["state"] == "LASER_STATE_ON", "equal timestamp regressed");
            f.clock.advance(1.1);
            check(f.engine.snapshot()["lasers"]["201"]["state"] == "LASER_STATE_UNKNOWN", "stale state displayed");
        });
        test("STOP cancels pending laser ON retries and sends OFF", [] {
            Fixture f; f.aim();
            f.peer().laser_supported = true;
            f.peer().cap->set_laser_control_supported(true);
            f.engine.command("laser on 201 1000");
            f.engine.command("stop 201");
            auto q = f.transport.last(pb::Envelope::kLaserCommand).laser_command();
            check(q.operation() == pb::LaserCommand::OPERATION_OFF && q.command_sequence() == 2 &&
                      !q.has_duration_ms(), "STOP did not supersede ON");
            f.clock.advance(.21);
            f.engine.requests.tick(f.engine.devices.peers);
            check(f.transport.last(pb::Envelope::kLaserCommand).laser_command().operation() ==
                      pb::LaserCommand::OPERATION_OFF, "ON retried after STOP");
        });
        test("ACCEPTED suppresses retry but keeps completion deadline", [] {
            Fixture f;
            int n = 0;
            auto q = f.point();
            f.engine.requests.request(f.peer(), q, pb::Envelope::kPointingResult,
                                      [&](auto *) { ++n; });
            f.result(q, pb::REQUEST_RESULT_ACCEPTED);
            auto count = f.transport.packets.size();
            f.clock.advance(.8);
            f.engine.requests.tick(f.engine.devices.peers);
            check(n == 0 && f.transport.packets.size() == count, "accepted isn't completion");
            f.clock.advance(1.3);
            f.engine.requests.tick(f.engine.devices.peers);
            check(n == 1, "completion timeout");
        });
        test("wrong sequence and session cannot complete", [] {
            Fixture f;
            int n = 0;
            auto q = f.point();
            f.engine.requests.request(f.peer(), q, pb::Envelope::kPointingResult,
                                      [&](auto *) { ++n; });
            auto r = f.reply(q);
            r.mutable_pointing_result()->set_command_sequence(6);
            f.engine.requests.result(f.peer(), r);
            auto p = f.peer();
            p.session = uid();
            f.engine.requests.result(p, f.reply(q));
            check(n == 0, "wrong correlation accepted");
            f.result(q);
            check(n == 1, "matching reply rejected");
        });
        test("cancelled POINT never retries or accepts late result", [] {
            Fixture f;
            int n = 0;
            auto q = f.point();
            f.engine.requests.request(f.peer(), q, pb::Envelope::kPointingResult,
                                      [&](auto *) { ++n; });
            f.engine.requests.cancel("201");
            f.clock.advance(5);
            f.engine.requests.tick(f.engine.devices.peers);
            f.result(q);
            check(n == 0 && f.transport.packets.size() == 1, "cancelled request revived");
        });
        test("replaced connection expires without retry", [] {
            Fixture f;
            int n = 0;
            auto q = f.point();
            f.engine.requests.request(f.peer(), q, pb::Envelope::kPointingResult, [&](auto *e) {
                if (!e)
                    ++n;
            });
            f.peer().session = uid();
            f.clock.advance(.201);
            f.engine.requests.tick(f.engine.devices.peers);
            check(n == 1 && f.transport.packets.size() == 1, "old connection retried");
        });
        test("wall clock adjustment cannot trigger retry deadline", [] {
            Fixture f;
            auto q = f.point();
            f.engine.requests.request(f.peer(), q, pb::Envelope::kPointingResult, [](auto *) {});
            f.clock.wall += 60;
            f.engine.requests.tick(f.engine.devices.peers);
            check(f.transport.packets.size() == 1, "wall time used as deadline");
            auto e = f.transport.last(pb::Envelope::kPointingCommand);
            check(e.sent_at() == utc(1700000000), "injected wall timestamp ignored");
        });
        test("timeout callback can enqueue STOP", [] {
            Fixture f;
            auto q = f.point();
            f.engine.requests.request(
                f.peer(), q, pb::Envelope::kPointingResult,
                [&](auto *) {
                    auto stop = f.point();
                    stop.mutable_pointing_command()->set_operation(
                        pb::PointingCommand::OPERATION_STOP);
                    f.engine.requests.request(f.peer(), stop, pb::Envelope::kPointingResult,
                                              [](auto *) {});
                },
                .1);
            f.clock.advance(.11);
            f.engine.requests.tick(f.engine.devices.peers);
            check(f.engine.requests.size() == 1 &&
                      f.transport.count(pb::Envelope::kPointingCommand) == 2,
                  "callback mutation lost pending");
        });
        test("duplicate and older status do not renew angle freshness", [] {
            Fixture f;
            auto before = f.peer().status_received;
            auto s = *f.peer().status;
            f.clock.advance(.5);
            s.mutable_angles()->set_pan_deg(80);
            f.engine.devices.status(f.peer(), s);
            check(f.peer().status_received == before && f.peer().status->angles().pan_deg() == 0,
                  "duplicate refreshed state");
            s.mutable_angles()->set_sample_sequence(0);
            f.engine.devices.status(f.peer(), s);
            check(f.peer().status_received == before, "old state refreshed");
        });
        test("heartbeat uses uint64 sequence and duplicates do not renew liveness", [] {
            Fixture f;
            pb::Heartbeat h;
            h.set_boot_id(f.peer().boot);
            h.set_sequence(uint64_t(1) << 40);
            f.engine.devices.heartbeat(f.peer(), h);
            double alive = f.peer().alive;
            f.clock.advance(.5);
            f.engine.devices.heartbeat(f.peer(), h);
            check(f.peer().hb == (uint64_t(1) << 40) && f.peer().alive == alive,
                  "heartbeat duplicate refreshed liveness");
        });
        test("injected Strategy determines transmitted angles", [] {
            Fixture f;
            f.aim();
            auto q = f.transport.last(pb::Envelope::kPointingCommand).pointing_command();
            check(q.pan_deg() == 13 && q.tilt_deg() == 7, "solver not used");
        });
        test("replacement Strategy cannot bypass device limits", [] {
            Fixture f;
            f.solver.pan = 999;
            f.aim();
            check(f.transport.last(pb::Envelope::kPointingCommand).pointing_command().operation() ==
                      pb::PointingCommand::OPERATION_STOP,
                  "limit bypassed");
        });
        test("next aim waits for terminal result", [] {
            Fixture f;
            f.aim();
            auto q = f.transport.last(pb::Envelope::kPointingCommand);
            f.result(q, pb::REQUEST_RESULT_ACCEPTED);
            f.clock.advance(.201);
            f.fresh();
            f.fresh("101");
            f.observed(2);
            f.engine.effector.tick(f.clock.monotonic());
            check(f.transport.count(pb::Envelope::kPointingCommand) == 1,
                  "accepted advanced stream");
            f.result(q);
            f.engine.effector.tick(f.clock.monotonic());
            check(f.transport.count(pb::Envelope::kPointingCommand) == 2,
                  "completed didn't release stream");
        });
        test("STOP needs result and newer measured stopped feedback", [] {
            Fixture f;
            f.aim();
            auto old = f.transport.last(pb::Envelope::kPointingCommand);
            f.engine.effector.stop("201");
            auto stop = f.transport.last(pb::Envelope::kPointingCommand);
            check(stop.pointing_command().command_sequence() >
                      old.pointing_command().command_sequence(),
                  "STOP fence not newer");
            f.result(stop);
            f.engine.effector.tick(f.clock.monotonic());
            check(!f.engine.effector.stopped(), "old feedback confirmed STOP");
            f.fresh();
            f.engine.effector.tick(f.clock.monotonic());
            check(f.engine.effector.stopped(), "fresh stopped feedback ignored");
        });
        test("OBS waits for EFF stop before collection stop", [] {
            Fixture f;
            f.aim();
            f.engine.observer.point(10, 5);
            f.engine.observer.tick(f.clock.monotonic());
            check(f.engine.observer.action->phase == ObserverPhase::WaitEffStop &&
                      f.transport.count(pb::Envelope::kMeasurementCommand) == 0,
                  "OBS skipped EFF stop");
            f.result(f.transport.last(pb::Envelope::kPointingCommand));
            f.fresh();
            f.engine.effector.tick(f.clock.monotonic());
            f.engine.observer.tick(f.clock.monotonic());
            check(f.engine.observer.action->phase == ObserverPhase::WaitCollectionStop,
                  "OBS didn't progress after stop");
        });
        test("OBS motion completion alone cannot resume collection", [] {
            Fixture f;
            f.obs_moving();
            auto q = f.transport.last(pb::Envelope::kObsMotionCommand);
            auto before = f.transport.count(pb::Envelope::kMeasurementCommand);
            f.result(q);
            f.engine.observer.tick(f.clock.monotonic());
            check(f.transport.count(pb::Envelope::kMeasurementCommand) == before,
                  "collection resumed on old angle sample");
            f.fresh("101", pb::MEASUREMENT_STATE_STOPPED);
            f.engine.observer.tick(f.clock.monotonic());
            check(f.engine.observer.action->phase == ObserverPhase::Collecting,
                  "fresh stop did not permit collection");
        });
        test("lost OBS completion issues STOP and never auto collects", [] {
            Fixture f;
            f.obs_moving();
            f.clock.advance(.61);
            f.engine.requests.tick(f.engine.devices.peers);
            f.clock.advance(.21);
            f.engine.requests.tick(f.engine.devices.peers);
            f.clock.advance(.21);
            f.engine.requests.tick(f.engine.devices.peers);
            auto q = f.transport.last(pb::Envelope::kObsMotionCommand);
            check(q.obs_motion_command().operation() == pb::ObsMotionCommand::OPERATION_STOP &&
                      f.engine.observer.action->failed,
                  "lost result did not issue STOP");
        });
        test("late replaced OBS motion result cannot restart collection", [] {
            Fixture f;
            f.obs_moving();
            auto q = f.transport.last(pb::Envelope::kObsMotionCommand);
            f.engine.observer.stop();
            f.result(q);
            check(!f.engine.observer.action->ack && f.engine.observer.action->operation == "STOP",
                  "late completion altered replacement");
        });
        test("terminal OBS operation cannot resurrect", [] {
            ObserverOperation a;
            a.phase = ObserverPhase::Completed;
            bool rejected = false;
            try {
                a.transition(ObserverPhase::Moving);
            } catch (const std::invalid_argument &) {
                rejected = true;
            }
            check(rejected, "terminal operation revived");
        });
        test("relative position atan2 matches preserved geometry", [] {
            auto c = config();
            RelativePointingSolver solver;
            auto [pan, tilt] =
                solver.solve({"id", .2, .1, 3, 48}, c.devices.at("101"), c.devices.at("201"));
            check(std::abs(pan - std::atan2(.1, 3) * 180 / std::acos(-1)) < 1e-9, "pan changed");
            check(std::abs(tilt - std::atan2(.1, std::hypot(3, .1)) * 180 / std::acos(-1)) < 1e-9,
                  "tilt changed");
        });
        test("effector mounting offset shifts the pointing origin", [] {
            auto c = config();
            auto &obs = c.devices.at("101");
            auto &eff = c.devices.at("201");
            obs.position = {10, 20, 30};
            obs.rotation = {1, 0, 0, 0, 1, 0, 0, 0, 1};
            eff.position = {12, 22, 32};
            eff.offset = {.5, -.25, 1};
            // Sensor origin is (12.5, 21.75, 33); the object is (3, 4, 5) beyond it.
            RelativePointingSolver solver;
            auto [pan, tilt] = solver.solve({"id", 5.5, 5.75, 8, 48}, obs, eff);
            check(std::abs(pan - 53.13010235415598) < 1e-9, "effector offset pan ignored");
            check(std::abs(tilt - 45) < 1e-9, "effector offset tilt ignored");
        });
        test("effector mounting offset follows the installation rotation", [] {
            auto c = config();
            auto &obs = c.devices.at("101");
            auto &eff = c.devices.at("201");
            obs.rotation = {1, 0, 0, 0, 1, 0, 0, 0, 1};
            eff.position = {2, 3, 4};
            eff.rotation = {0, -1, 0, 1, 0, 0, 0, 0, 1};
            eff.offset = {1, 2, 3};
            // A 90-degree installation rotation puts the sensor at (0, 4, 7).
            // The object's displacement from it is (4, 3, 5) in effector axes.
            RelativePointingSolver solver;
            auto [pan, tilt] = solver.solve({"id", -3, 8, 12, 48}, obs, eff);
            check(std::abs(pan - 36.86989764584402) < 1e-9, "effector offset frame wrong");
            check(std::abs(tilt - 45) < 1e-9, "rotated effector offset tilt wrong");
        });
        test("observation and effector mounting offsets compose", [] {
            auto c = config();
            auto &obs = c.devices.at("101");
            auto &eff = c.devices.at("201");
            obs.offset = {.5, 1, 2};
            eff.position = {12, 22, 32};
            eff.rotation = {0, -1, 0, 1, 0, 0, 0, 0, 1};
            eff.offset = {1, 2, 3};
            // OBS mounting maps this object to (6, 26, 40) in project coordinates.
            // FCS sensor origin is (10, 23, 35), leaving local displacement (3, 4, 5).
            RelativePointingSolver solver;
            auto [pan, tilt] = solver.solve({"id", 25.5, 39, 4, 48}, obs, eff);
            check(std::abs(pan - 53.13010235415598) < 1e-9, "combined offsets pan wrong");
            check(std::abs(tilt - 45) < 1e-9, "combined offsets tilt wrong");
        });
        test("expired acquisition profile blocks latest", [] {
            Fixture f;
            auto &p = f.peer("101");
            p.mapping = {{"valid_until", f.clock.time() - 1}};
            bool rejected = false;
            try {
                f.engine.observation.latest();
            } catch (const std::invalid_argument &) {
                rejected = true;
            }
            check(rejected, "expired profile accepted");
        });
        test("clock evidence is bound to connection and boot", [] {
            Fixture f;
            Json m = {{"boot_id", uid()},
                      {"connection_session_id", f.peer("101").session},
                      {"evidence", "measured"},
                      {"offset_seconds", 0},
                      {"uncertainty_ms", 1}};
            bool rejected = false;
            try {
                f.engine.observation.trust_clock("101", m);
            } catch (const std::invalid_argument &) {
                rejected = true;
            }
            check(rejected, "foreign boot evidence accepted");
        });
        test("wall clock jump invalidates observations and stops aim", [] {
            Fixture f;
            f.aim();
            f.clock.wall += 60;
            f.engine.tick();
            check(f.engine.observation.cloud.state == "CLOCK_CHANGED" &&
                      f.engine.effector.sessions.at("201")->state == EffectorPhase::Stopping,
                  "clock jump did not invalidate");
        });
        test("manual Engine constructs and ticks without sockets or threads", [] {
            Fixture f;
            f.engine.tick();
            check(f.engine.snapshot().contains("devices"), "manual engine failed");
        });
        std::cout << passed << " native behavior cases passed\n";
    } catch (const std::exception &e) {
        std::cerr << "FAIL after " << passed << " cases: " << e.what() << '\n';
        return 1;
    }
}
