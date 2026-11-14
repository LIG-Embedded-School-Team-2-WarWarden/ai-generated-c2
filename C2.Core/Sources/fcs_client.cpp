#include "c2/fcs_client.hpp"

#include <stdexcept>
#include <utility>
#include <sstream>
#include <iomanip>

namespace c2::fcs {
namespace {
std::string packet_info(const Packet& p) {
    std::ostringstream out;
    out << "id=0x" << std::hex << std::uppercase << unsigned(p.id) << std::dec
        << " sequence=" << p.sequence << " reply_to=" << p.reply_to;
    return out.str();
}
std::string wire_hex(const std::vector<std::byte>& bytes) {
    std::ostringstream out;
    out << std::hex << std::setfill('0');
    for (std::size_t i=0; i<bytes.size() && i<64; ++i)
        out << std::setw(2) << std::to_integer<unsigned>(bytes[i]);
    if(bytes.size()>64) out << "...";
    return out.str();
}
}
void Client::diagnose(const std::string& message) const noexcept {
    try { if(config_.diagnostic) config_.diagnostic("FCS wire "+message); } catch (...) {}
}
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
    last_response_sequence_.reset();
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
    return exchange_locked(request);
}
std::optional<Packet> Client::exchange_locked(Packet request, bool session_probe) {
    std::unique_lock lock(mutex_);
    if (stopped_) throw std::logic_error("FCS client is stopped");
    if (request.id != MessageId::connect && !state_.connected && !session_probe)
        throw std::logic_error("FCS handshake not complete");
    const bool was_connected = state_.connected;
    request.sequence = next_sequence_;
    next_sequence_ = static_cast<std::uint16_t>(next_sequence_ + 1);
    pending_ = request;
    response_.reset();
    response_deadline_ = std::chrono::steady_clock::now() + config_.response_timeout;
    const auto started=std::chrono::steady_clock::now();
    const auto context=packet_info(request);
    try {
        // Receive callback may run immediately; pending is installed before send.
        const auto bytes=encode(request);
        diagnose("tx "+context+" peer="+config_.peer.address+":"+std::to_string(config_.peer.port)+
            " timeout_ms="+std::to_string(config_.response_timeout.count())+" hex="+wire_hex(bytes));
        transport_.send(bytes, config_.peer);
    } catch (...) {
        const auto stats=transport_.stats();
        try { throw; } catch(const std::exception& error) {
            diagnose("send_failed "+context+" error="+error.what()+
                " socket_error="+std::to_string(stats.last_socket_error));
        } catch(...) { diagnose("send_failed "+context+" error=unknown_exception"); }
        pending_.reset();
        if (request.id == MessageId::connect) state_.connected = false;
        else if (request.id == MessageId::status && !session_probe &&
                 ++state_.missed_status >= config_.missed_status_limit)
            state_.connected = false;
        throw;
    }
    received_.wait_until(lock, response_deadline_,
        [this] { return stopped_ || response_.has_value(); });
    auto reply = stopped_ ? std::optional<Packet>{} : response_;
    diagnose(std::string(reply?"complete ":stopped_?"stopped ":"timeout outcome_unknown ")+context+
        " elapsed_ms="+std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now()-started).count()));
    if(!reply && !stopped_) {
        const auto stats=transport_.stats();
        diagnose("timeout_transport "+context+" receive_errors="+std::to_string(stats.receive_errors)+
            " handler_errors="+std::to_string(stats.handler_errors)+
            " socket_error="+std::to_string(stats.last_socket_error));
    }
    pending_.reset(); response_.reset();
    if (stopped_) return {};
    if (request.id == MessageId::connect) {
        // EF01 from the same UDP endpoint does NOT clear the firmware's
        // request history. Only a fresh FE02 proves the new request stream.
        state_.connected = false;
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
        } else if (!session_probe) {
            if (++state_.missed_status >= config_.missed_status_limit)
                state_.connected = false;
            diagnose("status_missed count="+std::to_string(state_.missed_status)+
                " limit="+std::to_string(config_.missed_status_limit)+
                " last_status_age_ms="+(state_.status?std::to_string(
                    std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now()-state_.status_received_at).count()):"unknown"));
        }
    }
    if (state_.connected != was_connected) {
        const auto reason=state_.connected?"fresh_status_confirmed":
            request.id==MessageId::connect?"session_revalidation":
            reply?"device_reported_disconnected":"consecutive_status_timeouts";
        diagnose("connection_changed connected="+std::to_string(state_.connected)+
            " reason="+reason+" "+context);
    }
    return reply;
}
void Client::receive(std::vector<std::byte> bytes, const Endpoint& source) {
    const auto wire=" source="+source.address+":"+std::to_string(source.port)+
        " bytes="+std::to_string(bytes.size())+" hex="+wire_hex(bytes);
    if (source.address != config_.peer.address || source.port != config_.peer.port) {
        diagnose("rx_drop reason=unexpected_peer"+wire); return;
    }
    Packet packet;
    try { packet = decode(bytes); } catch (const std::invalid_argument& error) {
        diagnose("rx_drop reason=decode_error error="+std::string(error.what())+wire); return;
    }
    std::lock_guard lock(mutex_);
    const char* reason=stopped_?"stopped":!pending_?"no_pending_request":response_?"duplicate_response":
        std::chrono::steady_clock::now()>=response_deadline_?"deadline_expired":
        packet.id!=response_id(pending_->id)?"unexpected_message_id":
        packet.reply_to!=pending_->sequence?"reply_to_mismatch":
        packet.id!=MessageId::connect_reply && last_response_sequence_ &&
            (std::uint16_t(packet.sequence-*last_response_sequence_)==0 ||
             std::uint16_t(packet.sequence-*last_response_sequence_)>=32768)?"stale_response_sequence":nullptr;
    if(reason) {
        diagnose("rx_drop reason="+std::string(reason)+" "+packet_info(packet)+
            (pending_?" pending_"+packet_info(*pending_):"")+wire); return;
    }
    static constexpr const char* result_names[]={"accepted","busy","invalid_value","device_error"};
    diagnose("rx_match "+packet_info(packet)+
        (packet.id==MessageId::status_reply?" flags="+std::to_string(packet.flags):
            " result="+std::to_string(unsigned(packet.result))+" result_name="+result_names[unsigned(packet.result)])+wire);
    response_ = packet;
    last_response_sequence_ = packet.sequence;
    received_.notify_one();
}
std::optional<Packet> Client::connect() {
    // Serialize the entire handshake/probe transaction against control commands.
    std::lock_guard request_lock(request_mutex_);
    auto handshake=exchange_locked(Packet{MessageId::connect});
    if(!handshake || handshake->result!=Result::accepted) return handshake;
    // Firmware accepts only forward TxSeq values (16-bit half-range ordering),
    // and retains 256 replies for the same peer across ECS restarts. Four
    // read-only probes spaced one quarter of the sequence ring cover any old
    // high-water mark. Cached FE02 replies precede FE01's fresh response seq
    // and are rejected in receive(). Never probe with a mutating command.
    for(unsigned attempt=0; attempt<4; ++attempt) {
        {
            std::lock_guard lock(mutex_);
            if(stopped_) return {};
            if(attempt) next_sequence_=std::uint16_t(next_sequence_+16383);
        }
        diagnose("session_probe attempt="+std::to_string(attempt+1));
        auto status=exchange_locked(Packet{MessageId::status},true);
        if(status) {
            if(status->flags&1) {
                diagnose("session_ready request_sequence="+std::to_string(status->reply_to));
                return handshake;
            }
            break;
        }
    }
    diagnose("session_sync_failed control_commands_blocked");
    return {};
}
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
