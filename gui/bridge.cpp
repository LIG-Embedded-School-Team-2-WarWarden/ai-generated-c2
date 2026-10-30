#include "bridge.hpp"
#include "ipc_server.hpp"
#include <algorithm>
#include <cmath>
#include <numbers>

namespace c2mvp {
namespace {
namespace wire = c2::gui;
std::uint32_t code(wire::Result r) { return static_cast<std::uint32_t>(r); }
int azimuth(double pan) { return static_cast<int>(std::lround(std::clamp(135.0 - pan, 0.0, 270.0))); }
bool displayable(const Peer& p) {
    return p.status && p.status->angles().has_pan_deg() && p.status->angles().has_tilt_deg() &&
           std::abs(p.status->angles().pan_deg()) <= 135;
}
}
GuiBridge::GuiBridge(Engine& e, bool open_transport) : engine(e) {
    for (const auto& [id, cfg] : e.config.devices)
        if (cfg.role == "EFF") {
            require(effector.empty(), "GUI v2 supports exactly one EFF");
            effector = id;
        }
    require(!effector.empty(), "GUI v2 requires an EFF installation");
    if (open_transport) {
        transport = c2_gui_create(wire::mapping_name, wire::mutex_name, wire::pipe_path);
        require(transport != nullptr, c2_gui_error());
    }
}
GuiBridge::~GuiBridge() { if (transport) c2_gui_destroy(transport); }
void GuiBridge::refresh() {
    auto obs = engine.devices.peers.find(engine.config.observer);
    auto session = obs == engine.devices.peers.end() ? std::string{} : obs->second.session;
    if (session != observer_session) { ids.clear(); observer_session = session; }
    visible.clear();
    auto& cloud = engine.observation.cloud;
    if (cloud.result && engine.clock.monotonic() - cloud.result->received <= 1 &&
        (cloud.state == "CURRENT" || cloud.state == "DISPLAY_ONLY" ||
         cloud.state == "UPDATING" || cloud.state == "PROCESSING")) {
        for (const auto& object : cloud.result->objects) {
            if (visible.size() == wire::target_capacity) break;
            if (!ids.contains(object.id)) {
                require(next_id != 0, "GUI target ID exhausted");
                ids[object.id] = next_id++;
            }
            visible[ids.at(object.id)] = object.id;
        }
    }
    if (!target) return;
    auto c = engine.effector.sessions.find(effector);
    auto p = engine.devices.peers.find(effector);
    if (c == engine.effector.sessions.end() || c->second->session != assigned_session ||
        p == engine.devices.peers.end() || !engine.devices.valid(p->second)) {
        assignment = 3; fire = 0; return;
    }
    const auto& control = *c->second;
    if (cancelling) {
        if (control.state == EffectorPhase::Stopped && control.stop_ack) {
            target = assignment = fire = 0; assigned_session.clear(); cancelling = false;
        } else if (control.reason == "STOP_UNCONFIRMED") assignment = 3;
        return;
    }
    if (control.state == EffectorPhase::Stopped || control.state == EffectorPhase::Stopping) {
        assignment = 3; fire = 0; return;
    }
    assignment = 1;
    if (control.state == EffectorPhase::Running && control.sequence > 0 &&
        !engine.requests.has(effector, pb::Envelope::kPointingResult) &&
        engine.devices.valid(p->second, true)) {
        try {
            const auto& latest = engine.observation.latest();
            require(latest.objects.size() == 1 && latest.objects.front().id == control.object,
                    "assigned object no longer current");
            RelativePointingSolver solver;
            auto [pan, tilt] = solver.solve(latest.objects.front(),
                engine.config.devices.at(engine.config.observer), engine.config.devices.at(effector));
            if (std::abs(p->second.status->angles().pan_deg() - pan) <= 1 &&
                std::abs(p->second.status->angles().tilt_deg() - tilt) <= 1) assignment = 2;
        } catch (const std::exception&) { }
    }
    if (laser_sequence && control.laser_sequence == laser_sequence) {
        if (control.laser_result == "REQUEST_RESULT_COMPLETED") fire = 1;
        else if (control.laser_result != "PENDING") { assignment = 3; fire = 0; }
    }
}
wire::Frame GuiBridge::frame() {
    refresh();
    wire::Frame out;
    out.header.sequence = ++sequence;
    out.header.timestamp_ms = static_cast<std::uint64_t>(engine.clock.time() * 1000);
    auto& payload = out.payload;
    payload.rdr_count = payload.fcs_count = payload.selected_rdr = payload.selected_fcs = 1;
    auto& r = payload.rdr; auto& f = payload.fcs;
    r.id = f.id = 1;
    const auto& obs_cfg = engine.config.devices.at(engine.config.observer);
    const auto& eff_cfg = engine.config.devices.at(effector);
    // Engine world axes: forward/left/up. GUI map axes: right/forward.
    r.position_x = static_cast<float>(-obs_cfg.position[1]);
    r.position_y = static_cast<float>(obs_cfg.position[0]);
    f.position_x = static_cast<float>(-eff_cfg.position[1]);
    f.position_y = static_cast<float>(eff_cfg.position[0]);
    r.azimuth = f.azimuth = 135;
    // No measured FOV exists in ICD 1.0; zero avoids inventing a sensor footprint.
    r.field_of_view = 0;
    auto asset = [&](const std::string& id, auto& a) {
        auto found = engine.devices.peers.find(id);
        if (found == engine.devices.peers.end()) return;
        const auto& p = found->second;
        a.connected = engine.clock.monotonic() - p.alive <= 2;
        a.enabled = p.installed && engine.devices.valid(p) && displayable(p);
        if (p.status) {
            a.azimuth = azimuth(p.status->angles().pan_deg());
            a.elevation = static_cast<int>(std::lround(p.status->angles().tilt_deg()));
            // GUI v2 treats isOperating as asset operation availability, including
            // a ready/stopped motor (its mock uses g_available). It is NOT the
            // motor's MOVING flag; reporting MOVING here disables its ON button
            // exactly when an assigned pointer has reached its destination.
            a.operating = a.enabled;
        }
    };
    asset(engine.config.observer, r); asset(effector, f);
    auto lasers = engine.effector.lasers();
    f.laser_on = lasers.contains(effector) && lasers[effector]["state"] == "LASER_STATE_ON";
    if (engine.observation.cloud.result) {
        for (const auto& [id, object_id] : visible) {
            const auto& objects = engine.observation.cloud.result->objects;
            auto object = std::find_if(objects.begin(), objects.end(), [&](const auto& o) { return o.id == object_id; });
            if (object == objects.end()) continue;
            const auto& o = *object;
            std::array<double, 3> local{o.x + obs_cfg.offset[0], o.y + obs_cfg.offset[1], o.z + obs_cfg.offset[2]};
            std::array<double, 3> delta{};
            for (int i = 0; i < 3; ++i)
                for (int k = 0; k < 3; ++k) delta[i] += obs_cfg.rotation[3*i+k] * local[k];
            auto& t = payload.targets[payload.target_count++];
            t.id = id; t.rdr_id = 1; t.valid = 1;
            t.azimuth = azimuth(std::atan2(delta[1], delta[0]) * 180 / std::numbers::pi);
            t.elevation = static_cast<int>(std::lround(std::atan2(delta[2], std::hypot(delta[0],delta[1])) * 180 / std::numbers::pi));
            t.distance = static_cast<float>(std::sqrt(delta[0]*delta[0] + delta[1]*delta[1] + delta[2]*delta[2]));
            t.x = static_cast<float>(-(obs_cfg.position[1] + delta[1]));
            t.y = static_cast<float>(obs_cfg.position[0] + delta[0]);
            t.z = static_cast<float>(o.z);
            t.status = engine.observation.cloud.state == "DISPLAY_ONLY" ? 2 : 0;
        }
    }
    return out;
}
wire::PipeResponse GuiBridge::command(const wire::PipeCommand& q) {
    refresh();
    auto result = wire::Result::success;
    auto p = engine.devices.peers.find(effector);
    bool connected = p != engine.devices.peers.end() && engine.clock.monotonic() - p->second.alive <= 2;
    try {
        if (q.magic != wire::pipe_magic || q.version != wire::pipe_version || q.rdr_id != 1 || q.fcs_id != 1)
            result = wire::Result::invalid_request;
        else switch (static_cast<wire::Command>(q.command)) {
        case wire::Command::status: break;
        case wire::Command::assign:
            if (target) result = wire::Result::already_assigned;
            else if (!visible.contains(q.target_id)) result = wire::Result::asset_unavailable;
            else {
                engine.effector.begin(effector, visible.at(q.target_id));
                target = q.target_id; assigned_session = engine.effector.sessions.at(effector)->session;
                assignment = 1; fire = 0; laser_sequence = 0; cancelling = false;
            }
            break;
        case wire::Command::cancel:
            if (!target || q.target_id != target) result = wire::Result::not_assigned;
            else { engine.effector.stop(effector); cancelling = true; assignment = 1; }
            break;
        case wire::Command::fire:
            if (!target || q.target_id != target) result = wire::Result::not_assigned;
            else if (assignment != 2 || cancelling) result = wire::Result::asset_unavailable;
            else if (!p->second.laser_supported) result = wire::Result::not_supported;
            else if (!laser_sequence) {
                engine.effector.laser(effector, pb::LaserCommand::OPERATION_ON, 1000);
                laser_sequence = engine.effector.sessions.at(effector)->laser_sequence;
            }
            break;
        case wire::Command::set_rdr_elevation: {
            auto obs = engine.devices.peers.find(engine.config.observer);
            if (q.target_id > 90) result = wire::Result::invalid_request;
            else if (obs == engine.devices.peers.end() || !engine.devices.valid(obs->second))
                result = wire::Result::asset_unavailable;
            else engine.observer.point(obs->second.status->angles().pan_deg(), q.target_id);
            break;
        }
        default: result = wire::Result::not_supported; break;
        }
    } catch (const std::exception& e) {
        engine.log("gui_rejected", e.what()); result = wire::Result::asset_unavailable;
    }
    wire::PipeResponse out;
    out.request_id = q.request_id; out.command = q.command; out.result = code(result);
    out.fcs_id = 1; out.target_id = target; out.assignment = assignment; out.fire = fire; out.connected = connected;
    return out;
}
void GuiBridge::tick() {
    require(transport && c2_gui_healthy(transport), "GUI IPC stopped");
    wire::PipeCommand request; std::uint64_t ticket;
    if (c2_gui_poll(transport, &request, &ticket)) {
        auto response = command(request);
        require(c2_gui_reply(transport, &response, ticket) != 0, "GUI reply failed");
    }
    if (engine.clock.monotonic() - last_publish >= .05) {
        auto value = frame();
        require(c2_gui_publish(transport, &value) != 0, "GUI frame publish failed");
        last_publish = engine.clock.monotonic();
    }
}
}
