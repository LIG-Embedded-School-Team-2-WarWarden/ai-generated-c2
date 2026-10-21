#include "c2mvp/domain.hpp"
#include <regex>
namespace c2mvp {
static bool uuid_valid(const std::string &s) {
    static const std::regex re(
        R"([0-9a-fA-F]{8}-[0-9a-fA-F]{4}-[0-9a-fA-F]{4}-[0-9a-fA-F]{4}-[0-9a-fA-F]{12})");
    return std::regex_match(s, re);
}
void DeviceSessions::registration(const pb::Envelope &e, const Endpoint &source) {
    auto &r = e.register_request();
    require(uuid_valid(r.request_id()) && uuid_valid(r.boot_id()), "registration UUID invalid");
    auto cfg = host.config.devices.find(r.device_id());
    require(cfg != host.config.devices.end() && e.sender_id() == r.device_id() &&
                r.role() == (cfg->second.role == "OBS" ? pb::ROLE_OBS : pb::ROLE_EFF),
            "unlisted device/role");
    require(r.receive_port() > 0 && r.receive_port() <= 65535, "invalid receive port");
    require(!host.config.simulation || source.ip.starts_with("127."),
            "simulation allows loopback peers only");
    auto key = std::pair{r.device_id(), r.request_id()};
    auto raw = r.SerializeAsString();
    auto cache = registrations.find(key);
    auto make_reply = [&](bool conflict, const std::string &session) {
        pb::Envelope out;
        auto *reply = out.mutable_register_result();
        reply->set_device_id(r.device_id());
        reply->set_boot_id(r.boot_id());
        reply->set_c2_boot_id(host.boot);
        auto *result = reply->mutable_outcome();
        result->set_request_id(r.request_id());
        result->set_result(conflict ? pb::REQUEST_RESULT_REJECTED : pb::REQUEST_RESULT_COMPLETED);
        result->set_reported_at(host.clock.utc());
        result->set_completed_at(host.clock.utc());
        if (conflict)
            result->set_reason_code(pb::REASON_CODE_ID_CONFLICT);
        else
            reply->set_connection_session_id(session);
        host.transport.send(encode(envelope(out, r.device_id(), "", host.clock)),
                            {source.ip, static_cast<int>(r.receive_port())});
    };
    if (cache != registrations.end() && cache->second.raw != raw) {
        make_reply(true, "");
        return;
    }
    require(!retired[r.device_id()].contains(r.boot_id()), "retired boot registration");
    auto it = peers.find(r.device_id());
    Endpoint endpoint{source.ip, static_cast<int>(r.receive_port())};
    if (it != peers.end() && it->second.boot == r.boot_id())
        require(it->second.address == endpoint, "same boot changed endpoint");
    else {
        if (it != peers.end()) {
            retired[r.device_id()].insert(it->second.boot);
            host.effector.stop_all("DEVICE_RESTART");
            host.requests.cancel(r.device_id());
            if (r.device_id() == host.config.observer)
                host.observation.invalidate("DEVICE_RESTART", true);
            host.effector.sessions.erase(r.device_id());
        }
        Peer p;
        p.id = r.device_id();
        p.boot = r.boot_id();
        p.session = uid();
        p.address = endpoint;
        p.alive = host.clock.monotonic();
        peers[p.id] = std::move(p);
    }
    make_reply(false, peers.at(r.device_id()).session);
    registrations[key] = {raw, host.clock.monotonic()};
}
void DeviceSessions::heartbeat(Peer &p, const pb::Heartbeat &h) {
    if (h.boot_id() != p.boot || h.sequence() <= p.hb)
        return;
    p.hb = h.sequence();
    p.alive = host.clock.monotonic();
}
void DeviceSessions::status(Peer &p, const pb::DeviceStatus &s) {
    if (s.device_id() != p.id || s.angles().sample_sequence() <= p.sample)
        return;
    stamp(s.reported_at());
    p.sample = s.angles().sample_sequence();
    p.status = s;
    p.status_received = p.alive = host.clock.monotonic();
    if (p.install_result &&
        s.active_installation_revision() == host.config.devices.at(p.id).revision)
        p.installed = true;
    configure(p);
}
void DeviceSessions::capability(Peer &p, const pb::CapabilityReport &c) {
    require(c.device_id() == p.id, "capability device mismatch");
    if (p.cap && p.cap->SerializeAsString() != c.SerializeAsString()) {
        host.effector.stop_all("CAPABILITY_CHANGED");
        throw std::invalid_argument("re-register after capability change");
    }
    p.cap = c;
    configure(p);
}
bool DeviceSessions::valid(const Peer &p, bool stopped) const {
    if (!p.status || host.clock.monotonic() - p.status_received > 1 ||
        host.clock.monotonic() - p.alive > 2)
        return false;
    auto &s = *p.status;
    auto &a = s.angles();
    try {
        auto current = [&](const std::string &t) {
            double age = host.clock.time() - stamp(t);
            return age >= 0 && age <= 1;
        };
        return (s.device_state() == pb::DEVICE_STATE_IDLE ||
                s.device_state() == pb::DEVICE_STATE_BUSY) &&
               (!p.install_result ||
                s.active_installation_revision() == host.config.devices.at(p.id).revision) &&
               !s.has_fault_code() && s.estop_latch_state() == pb::EMERGENCY_LATCH_STATE_RELEASED &&
               (!s.has_estop_latched() || !s.estop_latched()) &&
               s.clock_state() == pb::CLOCK_STATE_SYNCED && s.has_clock_uncertainty_ms() &&
               finite(s.clock_uncertainty_ms()) && s.clock_uncertainty_ms() >= 0 &&
               s.clock_uncertainty_ms() <= host.config.uncertainty && current(s.reported_at()) &&
               a.sample_sequence() && a.pan_valid() && a.tilt_valid() && a.has_pan_deg() &&
               a.has_tilt_deg() && a.has_pan_measured_at() && a.has_tilt_measured_at() &&
               finite(a.pan_deg()) && finite(a.tilt_deg()) && current(a.pan_measured_at()) &&
               current(a.tilt_measured_at()) &&
               (!stopped || s.motion_state() == pb::MOTION_STATE_STOPPED);
    } catch (const std::exception &) {
        return false;
    }
}
void DeviceSessions::configure(Peer &p) {
    auto &cfg = host.config.devices.at(p.id);
    if (!p.cap || !valid(p, true))
        return;
    auto &cap = *p.cap;
    if (cap.calibration_id() != cfg.calibration)
        return;
    if (cfg.role == "OBS") {
        bool fixed = false;
        for (auto m : cap.supported_observation_modes())
            if (m == pb::OBSERVATION_MODE_FIXED_VIEW)
                fixed = true;
        if (!cap.organized_frame_supported() || cap.sensor_data_profile_id() != cfg.profile ||
            !cap.pose_feedback_supported() || !cap.lidar_measurement_stop_supported() || !fixed)
            return;
    }
    if (!p.install_sent) {
        pb::Envelope cmd;
        auto *q = cmd.mutable_installation_set();
        q->set_request_id(uid());
        q->set_device_id(p.id);
        q->set_installation_revision(cfg.revision);
        q->set_axis_calibration_id(cfg.calibration);
        for (auto v : cfg.rotation)
            q->add_mount_rotation(v);
        auto position = [](pb::Position *out, const std::array<double, 3> &v) {
            out->set_x(v[0]);
            out->set_y(v[1]);
            out->set_z(v[2]);
        };
        position(q->mutable_position_m(), cfg.position);
        position(q->mutable_sensor_offset_m(), cfg.offset);
        p.install_sent = true;
        auto id = p.id, session = p.session;
        host.requests.request(p, cmd, pb::Envelope::kInstallationResult,
                              [this, id, session](const pb::Envelope *e) {
                                  auto &p = peers.at(id);
                                  if (p.session != session)
                                      return;
                                  if (e && outcome(*e)->result() == pb::REQUEST_RESULT_COMPLETED &&
                                      e->installation_result().device_id() == id &&
                                      e->installation_result().active_installation_revision() ==
                                          host.config.devices.at(id).revision)
                                      p.install_result = true;
                              });
    }
    if (p.installed && cfg.role == "OBS" && !p.mode_sent) {
        p.mode_sent = true;
        p.obs_session = uid();
        pb::Envelope cmd;
        auto *q = cmd.mutable_obs_mode_command();
        q->set_request_id(uid());
        q->set_obs_control_session_id(p.obs_session);
        q->set_control_epoch(p.obs_epoch);
        q->set_observation_mode_revision(p.mode_revision);
        q->set_mode(pb::OBSERVATION_MODE_FIXED_VIEW);
        q->set_installation_revision(cfg.revision);
        auto id = p.id, session = p.session;
        unsigned revision = p.mode_revision;
        host.requests.request(p, cmd, pb::Envelope::kObsModeResult,
                              [this, id, session, revision](const pb::Envelope *e) {
                                  auto &p = peers.at(id);
                                  if (p.session != session)
                                      return;
                                  if (e) {
                                      auto &r = e->obs_mode_result();
                                      if (r.outcome().result() == pb::REQUEST_RESULT_COMPLETED &&
                                          r.obs_id() == id &&
                                          r.observation_mode_revision() == revision &&
                                          r.mode() == pb::OBSERVATION_MODE_FIXED_VIEW &&
                                          r.actual_state() == pb::MOTION_STATE_STOPPED) {
                                          p.mode_applied = stamp(r.mode_applied_at());
                                          p.mode = true;
                                          host.observer.collect(true);
                                      }
                                  }
                              });
    }
}
void DeviceSessions::tick() {
    double now = host.clock.monotonic();
    std::erase_if(registrations, [&](auto &kv) { return now - kv.second.received >= 60; });
}
} // namespace c2mvp
