#include "c2/fcs_client.hpp"

#include <stdexcept>
#include <utility>

namespace c2::fcs {
Client::Client(ClientConfig config)
    : config_(std::move(config)), transport_(config_.local,
          [this](auto bytes, auto source) { receive(std::move(bytes), source); }) {
    if (config_.peer.address.empty() || config_.peer.port == 0 ||
        config_.response_timeout.count() <= 0 || config_.missed_status_limit == 0)
        throw std::invalid_argument("invalid FCS client configuration");
}
Client::~Client() { stop(); }
void Client::start() {
    std::lock_guard request_lock(request_mutex_);
    transport_.start();
    std::lock_guard lock(mutex_);
    stopped_ = false;
    state_ = {};
}
void Client::stop() noexcept {
    {
        std::lock_guard lock(mutex_);
        stopped_ = true;
        state_.connected = false;
    }
    received_.notify_all();
    transport_.stop();
}
Endpoint Client::local_endpoint() const { return transport_.local_endpoint(); }
Snapshot Client::snapshot() const {
    std::lock_guard lock(mutex_);
    return state_;
}
std::optional<Packet> Client::exchange(Packet request) {
    std::lock_guard request_lock(request_mutex_);
    std::unique_lock lock(mutex_);
    if (stopped_) throw std::logic_error("FCS client is stopped");
    if (request.id != MessageId::connect && !state_.connected)
        throw std::logic_error("FCS handshake not complete");
    request.sequence = next_sequence_;
    next_sequence_ = static_cast<std::uint16_t>(next_sequence_ + 1);
    pending_ = request;
    response_.reset();
    response_deadline_ = std::chrono::steady_clock::now() + config_.response_timeout;
    try {
        // Receive callback may run immediately; pending is installed before send.
        transport_.send(encode(request), config_.peer);
    } catch (...) {
        pending_.reset();
        if (request.id == MessageId::connect) state_.connected = false;
        else if (request.id == MessageId::status &&
                 ++state_.missed_status >= config_.missed_status_limit)
            state_.connected = false;
        throw;
    }
    received_.wait_until(lock, response_deadline_,
        [this] { return stopped_ || response_.has_value(); });
    auto reply = stopped_ ? std::optional<Packet>{} : response_;
    pending_.reset(); response_.reset();
    if (stopped_) return {};
    if (request.id == MessageId::connect) {
        state_.connected = reply && reply->result == Result::accepted;
        state_.missed_status = 0;
        state_.status.reset();
    } else if (request.id == MessageId::status) {
        if (reply) {
            state_.missed_status = 0;
            const auto degrees = [](std::int16_t value) {
                return value == -32768 ? -32768.0 : angle_from_wire(value);
            };
            state_.status = Status{reply->flags, degrees(reply->elevation), degrees(reply->azimuth)};
            state_.status_received_at = std::chrono::steady_clock::now();
            state_.connected = (reply->flags & 1) != 0;
        } else if (++state_.missed_status >= config_.missed_status_limit) {
            state_.connected = false;
        }
    }
    return reply;
}
void Client::receive(std::vector<std::byte> bytes, const Endpoint& source) {
    if (source.address != config_.peer.address || source.port != config_.peer.port) return;
    Packet packet;
    try { packet = decode(bytes); } catch (const std::invalid_argument&) { return; }
    std::lock_guard lock(mutex_);
    if (stopped_ || !pending_ || response_ || std::chrono::steady_clock::now() >= response_deadline_ ||
        packet.id != response_id(pending_->id) || packet.reply_to != pending_->sequence) return;
    response_ = packet;
    received_.notify_one();
}
std::optional<Packet> Client::connect() { return exchange(Packet{MessageId::connect}); }
std::optional<Packet> Client::query() { return exchange(Packet{MessageId::status}); }
std::optional<Packet> Client::point(double elevation, double azimuth) {
    Packet request{MessageId::point};
    request.elevation = angle_to_wire(elevation); request.azimuth = angle_to_wire(azimuth);
    return exchange(request);
}
std::optional<Packet> Client::laser(bool on) {
    Packet request{MessageId::laser}; request.laser_state = on ? 1 : 0;
    return exchange(request);
}
std::optional<Packet> Client::demo() { return exchange(Packet{MessageId::demo}); }
}  // namespace c2::fcs
