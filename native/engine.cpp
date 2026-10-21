#include "c2mvp/domain.hpp"
#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>
namespace c2mvp {
Engine::Engine(Config cfg, Clock &c, DatagramTransport &t, PointDetector &d,
               const PointingSolver &s, bool worker)
    : config(std::move(cfg)), clock(c), transport(t),
      requests(c, t, [this](const auto &event, const auto &reason) { log(event, reason); }),
      devices(*this), observation(*this, d, worker), effector(*this, s), observer(*this) {
    require(!config.simulation || transport.address().ip.starts_with("127."),
            "synthetic clock profile requires loopback bind");
    last_wall = clock.time();
    last_mono = clock.monotonic();
}
void Engine::log(const std::string &event, const std::string &reason) {
    events.push_back({{"event", event}, {"reason", reason}});
    if (events.size() > 256)
        events.pop_front();
}
void Engine::receive(const Datagram &d) {
    auto e = decode(d.bytes);
    if (e.has_register_request()) {
        devices.registration(e, d.source);
        return;
    }
    auto it = devices.peers.find(e.sender_id());
    if (it == devices.peers.end() || it->second.session != e.connection_session_id() ||
        it->second.address.ip != d.source.ip) {
        if (it != devices.peers.end() && it->second.address.ip == d.source.ip) {
            pb::Envelope reply;
            reply.mutable_session_required()->set_device_id(it->second.id);
            reply.mutable_session_required()->set_c2_boot_id(boot);
            requests.send(it->second, reply);
        }
        throw std::invalid_argument("unknown session/source");
    }
    auto &p = it->second;
    switch (e.body_case()) {
    case pb::Envelope::kHeartbeat:
        devices.heartbeat(p, e.heartbeat());
        break;
    case pb::Envelope::kDeviceStatus:
        devices.status(p, e.device_status());
        break;
    case pb::Envelope::kCapabilityReport:
        devices.capability(p, e.capability_report());
        break;
    case pb::Envelope::kObsPose:
        if (p.id == config.observer) {
            auto &pose = e.obs_pose();
            auto old = p.poses.find(pose.pose_id());
            if (old != p.poses.end() &&
                old->second.SerializeAsString() != pose.SerializeAsString()) {
                observation.invalidate("POSE_CONFLICT");
                effector.stop_all("POSE_CONFLICT");
                return;
            }
            if (old == p.poses.end())
                p.pose_order.push_back(pose.pose_id());
            p.poses[pose.pose_id()] = pose;
            if (p.pose_order.size() > 64) {
                p.poses.erase(p.pose_order.front());
                p.pose_order.pop_front();
            }
        }
        break;
    case pb::Envelope::kPointCloudChunk:
        if (p.id == config.observer && p.installed && p.collecting &&
            e.point_cloud_chunk().collection_session_id() == p.collection) {
            bool verified = observation.verify(p, e.point_cloud_chunk());
            observation.cloud.ingest(p.session, e.point_cloud_chunk(), verified);
        }
        break;
    case pb::Envelope::kErrorReport:
        effector.stop_all("DEVICE_ERROR");
        break;
    default:
        if (outcome(e))
            requests.result(p, e);
        break;
    }
}
void Engine::tick() {
    double now = clock.monotonic(), wall = clock.time();
    if (std::abs((wall - last_wall) - (now - last_mono)) > .25) {
        for (auto &[id, p] : devices.peers)
            p.mapping = nullptr;
        observation.invalidate("CLOCK_CHANGED");
        effector.stop_all("CLOCK_CHANGED");
    }
    last_wall = wall;
    last_mono = now;
    if (now - last_hb >= .5) {
        last_hb = now;
        ++hb_sequence;
        for (auto &[id, p] : devices.peers) {
            pb::Envelope cmd;
            auto *h = cmd.mutable_heartbeat();
            h->set_boot_id(boot);
            h->set_sequence(hb_sequence);
            requests.send(p, cmd);
        }
    }
    requests.tick(devices.peers);
    devices.tick();
    observation.tick();
    effector.tick(now);
    observer.tick(now);
}
Json Engine::snapshot() const {
    Json result = Json::object();
    std::string state = observation.cloud.state;
    if (state == "CURRENT")
        try {
            observation.latest();
        } catch (const std::exception &) {
            state = "STALE";
        }
    result["devices"] = Json::object();
    for (auto &[id, p] : devices.peers) {
        Json pan = nullptr, tilt = nullptr;
        if (p.status) {
            if (p.status->angles().has_pan_deg())
                pan = p.status->angles().pan_deg();
            if (p.status->angles().has_tilt_deg())
                tilt = p.status->angles().tilt_deg();
        }
        result["devices"][id] = {{"session", p.session},
                                 {"boot", p.boot},
                                 {"installed", p.installed},
                                 {"connected", clock.monotonic() - p.alive <= 2},
                                 {"sample", p.sample},
                                 {"heartbeat", p.hb},
                                 {"angles_current", devices.valid(p)},
                                 {"collecting", p.collecting},
                                 {"measurement_result", p.measurement_result},
                                 {"pan", pan},
                                 {"tilt", tilt}};
    }
    result["observation"] = state;
    result["frame"] = nullptr;
    result["objects"] = Json::array();
    auto &r = observation.cloud.result;
    if (r) {
        result["frame"] = r->meta.frame_id();
        if (state == "CURRENT" || state == "DISPLAY_ONLY")
            for (auto &o : r->objects)
                result["objects"].push_back(
                    {{"id", o.id}, {"x", o.x}, {"y", o.y}, {"z", o.z}, {"count", o.count}});
    }
    result["controls"] = effector.snapshot();
    result["observer_action"] = observer.snapshot();
    result["pending"] = requests.size();
    return result;
}
Json Engine::command(const std::string &line) {
    // Windows path separators are literal; quoting only groups a path containing spaces.
    std::istringstream in(line);
    std::vector<std::string> args;
    std::string arg;
    while (in >> std::quoted(arg, '"', '\0'))
        args.push_back(arg);
    if (args.empty())
        return nullptr;
    auto &cmd = args[0];
    auto number = [](const std::string &s) {
        size_t end = 0;
        double n = std::stod(s, &end);
        require(end == s.size() && finite(n), "finite number required");
        return n;
    };
    if (cmd == "status" && args.size() == 1)
        return snapshot();
    if (cmd == "targets" && args.size() == 1) {
        auto s = snapshot();
        return {
            {"observation", s["observation"]}, {"frame", s["frame"]}, {"objects", s["objects"]}};
    }
    if (cmd == "events" && args.size() == 1)
        return Json(events);
    if (cmd == "aim" && (args.size() == 2 || args.size() == 3))
        effector.begin(args[1], args.size() == 3 ? args[2] : "");
    else if (cmd == "point" && args.size() == 4)
        effector.begin(args[1], "", std::pair{number(args[2]), number(args[3])});
    else if (cmd == "stop" && args.size() == 2)
        effector.stop(args[1]);
    else if (cmd == "collect" && args.size() == 2 && (args[1] == "start" || args[1] == "stop"))
        observer.collect(args[1] == "start");
    else if (cmd == "obs" && args.size() == 4 && args[1] == "point")
        observer.point(number(args[2]), number(args[3]));
    else if (cmd == "obs" && args.size() == 2 && args[1] == "stop")
        observer.stop();
    else if (cmd == "obs" && args.size() == 2 && args[1] == "status") {
        auto s = snapshot();
        return {{"device", s["devices"].contains(config.observer) ? s["devices"][config.observer]
                                                                  : Json(nullptr)},
                {"operation", s["observer_action"]},
                {"observation", s["observation"]}};
    } else if ((cmd == "clock-load" || cmd == "clock-profile") && args.size() == 3) {
        std::ifstream f(args[2]);
        require(f.good(), "cannot open evidence file");
        auto m = Json::parse(f);
        if (cmd == "clock-load")
            observation.trust_clock(args[1], m);
        else
            observation.trust_profile(args[1], m);
    } else
        throw std::invalid_argument("unknown command or arguments");
    return {{"accepted", cmd}};
}
} // namespace c2mvp
