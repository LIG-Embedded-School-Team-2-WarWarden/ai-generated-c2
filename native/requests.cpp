#include "c2mvp/domain.hpp"
namespace c2mvp {
std::string RequestTracker::send(const Peer &p, const pb::Envelope &cmd) {
    auto bytes = encode(envelope(cmd, p.id, p.session, clock));
    transport.send(bytes, p.address);
    return bytes;
}
void RequestTracker::request(const Peer &p, const pb::Envelope &cmd, pb::Envelope::BodyCase kind,
                             Callback cb, double timeout) {
    auto key = request_id(cmd);
    require(!key.empty() && !pending.contains(key), "duplicate/empty request ID");
    // Generated C++ messages are copied by value: caller mutation cannot alter correlation.
    Pending v;
    v.peer = p;
    v.command = cmd;
    v.kind = kind;
    v.callback = std::move(cb);
    v.timeout = timeout;
    v.packet = send(p, v.command);
    v.started = v.sent = clock.monotonic();
    pending.emplace(key, std::move(v));
    if (log)
        log("request", p.id + ":" + key);
}
bool RequestTracker::has(const std::string &id, pb::Envelope::BodyCase kind) const {
    for (auto &[k, v] : pending)
        if (v.peer.id == id && (kind == pb::Envelope::BODY_NOT_SET || v.kind == kind))
            return true;
    return false;
}
void RequestTracker::cancel(const std::string &id, std::set<pb::Envelope::BodyCase> kinds) {
    std::erase_if(pending, [&](auto &kv) {
        return kv.second.peer.id == id && (kinds.empty() || kinds.contains(kv.second.kind));
    });
}
void RequestTracker::result(const Peer &p, const pb::Envelope &e) {
    auto *out = outcome(e);
    if (!out)
        return;
    auto it = pending.find(out->request_id());
    if (it == pending.end())
        return;
    auto &v = it->second;
    if (v.peer.id != p.id || v.peer.session != p.session || v.kind != e.body_case())
        return;
    const auto &q = v.command;
    if (e.has_control_result()) {
        auto &r = e.control_result();
        auto &b = q.control_open();
        if (r.active_control_session_id() != b.control_session_id() ||
            r.control_epoch() != b.control_epoch())
            return;
    }
    if (e.has_pointing_result()) {
        auto &r = e.pointing_result();
        auto &b = q.pointing_command();
        if (r.control_session_id() != b.control_session_id() ||
            r.control_epoch() != b.control_epoch() || r.command_sequence() != b.command_sequence())
            return;
    }
    if (e.has_obs_motion_result()) {
        auto &r = e.obs_motion_result();
        auto &b = q.obs_motion_command();
        if (r.obs_id() != p.id || r.obs_control_session_id() != b.obs_control_session_id() ||
            r.control_epoch() != b.control_epoch() || r.command_sequence() != b.command_sequence())
            return;
    }
    if (out->has_detail() && out->detail().size() > 256)
        return;
    stamp(out->reported_at());
    if (out->result() == pb::REQUEST_RESULT_ACCEPTED) {
        v.accepted = true;
        return;
    }
    if (out->result() < pb::REQUEST_RESULT_COMPLETED ||
        out->result() > pb::REQUEST_RESULT_CANCELLED)
        return;
    if (!out->has_completed_at())
        return;
    stamp(out->completed_at());
    if (out->result() != pb::REQUEST_RESULT_COMPLETED && !out->has_reason_code())
        return;
    auto cb = std::move(v.callback);
    pending.erase(it);
    if (log)
        log("result", p.id + ":" + out->request_id());
    cb(&e);
}
void RequestTracker::tick(const std::map<std::string, Peer> &peers) {
    std::vector<std::string> keys;
    for (auto &[key, v] : pending)
        keys.push_back(key);
    for (auto &key : keys) {
        auto it = pending.find(key);
        if (it == pending.end())
            continue;
        auto &v = it->second;
        auto p = peers.find(v.peer.id);
        double now = clock.monotonic();
        if (p == peers.end() || p->second.session != v.peer.session ||
            now - v.started >= v.timeout ||
            (!v.accepted && now - v.sent >= .2 && v.attempts >= 3)) {
            auto cb = std::move(v.callback);
            pending.erase(it);
            cb(nullptr);
        } else if (!v.accepted && now - v.sent >= .2) {
            transport.send(v.packet, v.peer.address);
            v.sent = now;
            ++v.attempts;
        }
    }
}
} // namespace c2mvp
