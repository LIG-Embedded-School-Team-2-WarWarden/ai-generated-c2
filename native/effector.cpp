#include "c2mvp/domain.hpp"
#include <algorithm>
#include <cmath>
namespace c2mvp {
static bool completed(const pb::Envelope *e) {
    return e && outcome(*e)->result() == pb::REQUEST_RESULT_COMPLETED;
}
static void limits(const pb::CapabilityReport &cap, double pan, double tilt) {
    require(cap.has_pan_min_deg() && cap.has_pan_max_deg() && cap.has_tilt_min_deg() &&
                cap.has_tilt_max_deg() && finite(pan) && finite(tilt) && cap.pan_min_deg() <= pan &&
                pan <= cap.pan_max_deg() && cap.tilt_min_deg() <= tilt &&
                tilt <= cap.tilt_max_deg(),
            "OUT_OF_RANGE");
}
bool EffectorController::stopped() const {
    for (auto &[id, c] : sessions)
        if (c->state != EffectorPhase::Stopped)
            return false;
    return true;
}
void EffectorController::observed(const std::vector<Object> &objects, double now) {
    if (objects.size() == 1)
        for (auto &[id, c] : sessions)
            if (c->object == objects[0].id)
                c->last_observation = now;
}
void EffectorController::begin(const std::string &id, const std::string &object,
                               std::optional<std::pair<double, double>> manual) {
    auto &p = host.devices.peers.at(id);
    auto &cfg = host.config.devices.at(id);
    require(cfg.role == "EFF" && p.installed && host.devices.valid(p, true),
            "EFF not ready/stopped with current measured angles");
    require(p.cap.has_value(), "EFF capabilities missing");
    auto &cap = *p.cap;
    require(cap.absolute_pointing() && cap.measured_angle_feedback() && cap.stop_supported() &&
                cap.drain_supported(),
            "EFF capabilities missing");
    require(cap.has_point_completion_timeout_ms() && cap.has_stop_completion_timeout_ms() &&
                cap.point_completion_timeout_ms() > 0 &&
                cap.point_completion_timeout_ms() <= 60000 &&
                cap.stop_completion_timeout_ms() > 0 && cap.stop_completion_timeout_ms() <= 60000,
            "EFF timeouts missing/invalid");
    require(cap.has_pan_min_deg() && cap.has_pan_max_deg() && cap.has_tilt_min_deg() &&
                cap.has_tilt_max_deg() && finite(cap.pan_min_deg()) && finite(cap.pan_max_deg()) &&
                finite(cap.tilt_min_deg()) && finite(cap.tilt_max_deg()) &&
                cap.pan_min_deg() < cap.pan_max_deg() && cap.tilt_min_deg() < cap.tilt_max_deg(),
            "EFF limits missing/invalid");
    require(!sessions.contains(id) || sessions.at(id)->state == EffectorPhase::Stopped,
            "stop and confirm previous control first");
    std::string selected;
    if (manual)
        limits(cap, manual->first, manual->second);
    else {
        auto &r = host.observation.latest();
        require(r.objects.size() == 1 && (object.empty() || r.objects[0].id == object),
                "MVP requires one unambiguous object");
        selected = r.objects[0].id;
    }
    auto c = std::make_shared<EffectorSession>();
    c->session = uid();
    c->epoch = ++p.epoch;
    c->object = selected;
    c->manual = manual;
    c->last_observation = host.clock.monotonic();
    sessions[id] = c;
    pb::Envelope cmd;
    auto *q = cmd.mutable_control_open();
    q->set_request_id(uid());
    q->set_control_session_id(c->session);
    q->set_control_epoch(c->epoch);
    q->set_installation_revision(cfg.revision);
    q->set_mode(manual ? pb::CONTROL_MODE_MANUAL : pb::CONTROL_MODE_AUTO);
    host.requests.request(
        p, cmd, pb::Envelope::kControlResult, [this, id, c](const pb::Envelope *e) {
            if (!sessions.contains(id) || sessions.at(id) != c ||
                c->state != EffectorPhase::Opening)
                return;
            if (completed(e)) {
                c->state = c->manual ? EffectorPhase::Manual : EffectorPhase::Running;
                if (c->manual)
                    send_manual(host.devices.peers.at(id), c);
            } else
                stop(id, "CONTROL_UNCONFIRMED");
        });
}
void EffectorController::send_manual(Peer &p, const std::shared_ptr<EffectorSession> &c) {
    if (!host.devices.valid(p)) {
        stop(p.id, "DEVICE_UNAVAILABLE");
        return;
    }
    pb::Envelope cmd;
    auto *q = cmd.mutable_pointing_command();
    q->set_request_id(uid());
    q->set_control_session_id(c->session);
    q->set_control_epoch(c->epoch);
    q->set_command_sequence(++c->sequence);
    q->set_operation(pb::PointingCommand::OPERATION_POINT);
    q->set_issued_at(host.clock.utc());
    q->set_expires_at(host.clock.utc(1));
    q->set_pan_deg(c->manual->first);
    q->set_tilt_deg(c->manual->second);
    q->set_effector_installation_revision(host.config.devices.at(p.id).revision);
    q->set_origin(pb::CONTROL_MODE_MANUAL);
    auto id = p.id;
    host.requests.request(
        p, cmd, pb::Envelope::kPointingResult,
        [this, id, c](const pb::Envelope *e) {
            if (!completed(e))
                stop(id, "POINT_UNCONFIRMED");
            else if (c->state == EffectorPhase::Manual)
                c->reason = "POINT_COMPLETED";
        },
        p.cap->point_completion_timeout_ms() / 1000.0);
}
void EffectorController::stop(const std::string &id, const std::string &reason) {
    auto it = sessions.find(id);
    if (it == sessions.end())
        return;
    auto c = it->second;
    if (c->state == EffectorPhase::Stopped ||
        (c->state == EffectorPhase::Stopping && c->reason != "STOP_UNCONFIRMED"))
        return;
    auto &p = host.devices.peers.at(id);
    host.requests.cancel(id, {pb::Envelope::kLaserResult});
    if (p.laser_supported && p.cap && p.cap->laser_control_supported())
        laser(id, pb::LaserCommand::OPERATION_OFF);
    c->state = EffectorPhase::Stopping;
    c->reason = reason;
    c->stop_ack = false;
    c->stop_sample = p.sample;
    host.requests.cancel(id, {pb::Envelope::kControlResult, pb::Envelope::kPointingResult});
    pb::Envelope cmd;
    auto *q = cmd.mutable_pointing_command();
    q->set_request_id(uid());
    q->set_control_session_id(c->session);
    q->set_control_epoch(c->epoch);
    q->set_command_sequence(++c->sequence);
    q->set_operation(pb::PointingCommand::OPERATION_STOP);
    q->set_issued_at(host.clock.utc());
    host.requests.request(
        p, cmd, pb::Envelope::kPointingResult,
        [c](const pb::Envelope *e) {
            if (completed(e) &&
                e->pointing_result().actual_motion_state() == pb::MOTION_STATE_STOPPED)
                c->stop_ack = true;
            else
                c->reason = "STOP_UNCONFIRMED";
        },
        std::max(.2, p.cap ? p.cap->stop_completion_timeout_ms() / 1000.0 : 2.0));
}
void EffectorController::stop_all(const std::string &reason) {
    for (auto &[id, c] : sessions)
        stop(id, reason);
}
void EffectorController::laser(const std::string &id, pb::LaserCommand::Operation operation,
                                unsigned duration) {
    auto &p = host.devices.peers.at(id);
    require(host.config.devices.at(id).role == "EFF" && p.laser_supported && p.cap &&
                p.cap->laser_control_supported(), "EFF laser control unsupported");
    require(sessions.contains(id), "open EFF control with point/aim first");
    auto c = sessions.at(id);
    const bool on = operation == pb::LaserCommand::OPERATION_ON;
    if (on) {
        require((c->state == EffectorPhase::Manual || c->state == EffectorPhase::Running) &&
                    host.devices.valid(p, true), "EFF must be ready and measured stopped");
        require(duration > 0 && duration <= 5000, "laser duration must be 1..5000 ms");
        require(!host.requests.has(id, pb::Envelope::kLaserResult), "laser command pending");
    } else if (operation == pb::LaserCommand::OPERATION_OFF)
        host.requests.cancel(id, {pb::Envelope::kLaserResult});
    else
        require(!host.requests.has(id, pb::Envelope::kLaserResult), "laser command pending");
    pb::Envelope cmd;
    auto *q = cmd.mutable_laser_command();
    q->set_request_id(uid());
    q->set_control_session_id(c->session);
    q->set_control_epoch(c->epoch);
    q->set_command_sequence(++c->laser_sequence);
    q->set_operation(operation);
    q->set_issued_at(host.clock.utc());
    q->set_effector_installation_revision(host.config.devices.at(id).revision);
    if (on) {
        q->set_expires_at(host.clock.utc(1));
        q->set_duration_ms(duration);
    }
    c->laser_result = "PENDING";
    host.requests.request(p, cmd, pb::Envelope::kLaserResult,
        [this, id, c, operation](const pb::Envelope *e) {
            if (!sessions.contains(id) || sessions.at(id) != c)
                return;
            c->laser_result = e ? pb::RequestResult_Name(e->laser_result().outcome().result())
                                : "UNCONFIRMED";
            // State is displayed only from independently timestamped LaserStatus telemetry.
            if (!e || (completed(e) && operation != pb::LaserCommand::OPERATION_GET_STATUS &&
                (!e->laser_result().has_state_observed_at() ||
                 e->laser_result().actual_laser_state() !=
                     (operation == pb::LaserCommand::OPERATION_ON ? pb::LASER_STATE_ON : pb::LASER_STATE_OFF)))) {
                c->laser_status.reset();
                c->laser_result = "UNCONFIRMED";
                if (operation == pb::LaserCommand::OPERATION_ON)
                    stop(id, "LASER_UNCONFIRMED");
            }
        });
}
void EffectorController::laser_status(Peer &p, const pb::LaserStatus &s) {
    auto it = sessions.find(p.id);
    if (it == sessions.end() || s.device_id() != p.id)
        return;
    auto c = it->second;
    if (!s.has_control_session_id() || s.control_session_id() != c->session ||
        s.control_epoch() != c->epoch || s.last_accepted_sequence() > c->laser_sequence)
        return;
    double age = host.clock.time() - stamp(s.reported_at());
    if (age < -.1 || age > 1)
        return;
    if (s.has_state_observed_at()) {
        double measured_age = host.clock.time() - stamp(s.state_observed_at());
        if (measured_age < -.1 || measured_age > 1)
            return;
    }
    if (c->laser_status && (s.last_accepted_sequence() < c->laser_status->last_accepted_sequence() ||
        stamp(s.reported_at()) <= stamp(c->laser_status->reported_at())))
        return;
    c->laser_status = s;
    c->laser_received = host.clock.monotonic();
}
Json EffectorController::lasers() const {
    Json out = Json::object();
    for (const auto &[id, c] : sessions) {
        const auto &p = host.devices.peers.at(id);
        bool current = c->laser_status && c->laser_status->has_state_observed_at() &&
                       !c->laser_status->has_fault_code() &&
                       (c->laser_status->laser_state() == pb::LASER_STATE_ON ||
                        c->laser_status->laser_state() == pb::LASER_STATE_OFF) &&
                       host.clock.time() - stamp(c->laser_status->state_observed_at()) <= 1 &&
                       host.clock.monotonic() - c->laser_received <= 1 &&
                       host.clock.monotonic() - p.alive <= 2;
        out[id] = {{"supported", p.laser_supported && p.cap && p.cap->laser_control_supported()},
                   {"state", current ? pb::LaserState_Name(c->laser_status->laser_state()) : "LASER_STATE_UNKNOWN"},
                   {"result", c->laser_result}};
        if (current && c->laser_status->laser_state() == pb::LASER_STATE_ON &&
            c->laser_status->has_remaining_duration_ms())
            out[id]["remaining_duration_ms"] = std::max(0.0,
                c->laser_status->remaining_duration_ms() -
                (host.clock.monotonic() - c->laser_received) * 1000);
    }
    return out;
}
void EffectorController::tick(double now) {
    for (auto &[id, c] : sessions)
        update(id, c, now);
}
void EffectorController::update(const std::string &id, const std::shared_ptr<EffectorSession> &c,
                                double now) {
    auto &p = host.devices.peers.at(id);
    if (c->state == EffectorPhase::Stopping) {
        if (c->stop_ack && p.sample > c->stop_sample && host.devices.valid(p, true))
            c->state = EffectorPhase::Stopped;
        return;
    }
    if (c->state == EffectorPhase::Stopped)
        return;
    if (!host.devices.valid(p)) {
        stop(id, "DEVICE_UNAVAILABLE");
        return;
    }
    if (c->state == EffectorPhase::Opening || c->state == EffectorPhase::Manual)
        return;
    auto obs = host.devices.peers.find(host.config.observer);
    if (obs == host.devices.peers.end() || !host.devices.valid(obs->second, true)) {
        stop(id, "OBS_UNAVAILABLE");
        return;
    }
    auto &cloud = host.observation.cloud;
    if (cloud.state == "UPDATING" || cloud.state == "PROCESSING") {
        if (now - cloud.started > 1 || now - c->last_observation > 1)
            stop(id, "OBSERVATION_STALE");
        return;
    }
    try {
        auto &r = host.observation.latest();
        require(r.objects.size() == 1 && r.objects[0].id == c->object, "OBJECT_LOST_OR_AMBIGUOUS");
        auto [pan, tilt] = solver.solve(r.objects[0], host.config.devices.at(host.config.observer),
                                        host.config.devices.at(id));
        limits(*p.cap, pan, tilt);
        if (now - c->last_send < .2 || r.meta.frame_id() == c->last_frame ||
            host.requests.has(id, pb::Envelope::kPointingResult))
            return;
        pb::Envelope cmd;
        auto *q = cmd.mutable_pointing_command();
        q->set_request_id(uid());
        q->set_control_session_id(c->session);
        q->set_control_epoch(c->epoch);
        q->set_command_sequence(++c->sequence);
        q->set_operation(pb::PointingCommand::OPERATION_POINT);
        q->set_issued_at(host.clock.utc());
        q->set_expires_at(host.clock.utc(1));
        q->set_pan_deg(pan);
        q->set_tilt_deg(tilt);
        q->set_effector_installation_revision(host.config.devices.at(id).revision);
        q->set_origin(pb::CONTROL_MODE_AUTO);
        q->set_collection_session_id(r.meta.collection_session_id());
        q->set_object_id(c->object);
        q->set_source_device_id(host.config.observer);
        q->set_source_connection_session_id(obs->second.session);
        q->set_source_frame_id(r.meta.frame_id());
        q->set_observation_installation_revision(r.meta.installation_revision());
        host.requests.request(
            p, cmd, pb::Envelope::kPointingResult,
            [this, id](const pb::Envelope *e) {
                if (!completed(e))
                    stop(id, "POINT_UNCONFIRMED");
            },
            p.cap->point_completion_timeout_ms() / 1000.0);
        c->last_send = now;
        c->last_frame = r.meta.frame_id();
        host.log("point", id);
    } catch (const std::invalid_argument &e) {
        stop(id, e.what());
    }
}
Json EffectorController::snapshot() const {
    Json out = Json::object();
    for (auto &[id, c] : sessions)
        out[id] = {{"session", c->session},
                   {"epoch", c->epoch},
                   {"state", name(c->state)},
                   {"sequence", c->sequence},
                   {"object", c->object.empty() ? Json(nullptr) : Json(c->object)},
                   {"reason", c->reason},
                   {"stop_ack", c->stop_ack},
                   {"last_frame", c->last_frame}};
    return out;
}

} // namespace c2mvp
