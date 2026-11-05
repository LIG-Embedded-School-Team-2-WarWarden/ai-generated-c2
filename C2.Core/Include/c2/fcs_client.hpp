#pragma once

#include "c2/fcs_protocol.hpp"
#include "c2/udp_transport.hpp"

#include <chrono>
#include <condition_variable>
#include <mutex>
#include <optional>

namespace c2::fcs {
struct ClientConfig {
    Endpoint peer;
    UdpConfig local;
    std::chrono::milliseconds response_timeout{1000};
    std::uint32_t missed_status_limit{3};
};
struct Status {
    std::uint16_t flags{};
    // Degrees; preserve the unknown-angle sentinel for existing consumers.
    double elevation{}, azimuth{};
};
struct Snapshot {
    bool connected{};
    std::uint32_t missed_status{};
    std::optional<Status> status;
    std::chrono::steady_clock::time_point status_received_at{};
};

class Client final {
public:
    explicit Client(ClientConfig config);
    ~Client();
    void start();
    void stop() noexcept;
    [[nodiscard]] Endpoint local_endpoint() const;
    [[nodiscard]] Snapshot snapshot() const;
    // No automatic control-command retransmission. nullopt means outcome unknown.
    [[nodiscard]] std::optional<Packet> connect();
    [[nodiscard]] std::optional<Packet> query();
    // Public angles are in degrees; only the wire representation is validated.
    [[nodiscard]] std::optional<Packet> point(double elevation, double azimuth);
    [[nodiscard]] std::optional<Packet> laser(bool on);
    [[nodiscard]] std::optional<Packet> demo();
private:
    std::optional<Packet> exchange(Packet request);
    void receive(std::vector<std::byte> bytes, const Endpoint& source);
    ClientConfig config_;
    std::mutex request_mutex_;
    mutable std::mutex mutex_;
    std::condition_variable received_;
    bool stopped_{true};
    std::uint16_t next_sequence_{1};
    std::optional<Packet> pending_, response_;
    std::chrono::steady_clock::time_point response_deadline_{};
    Snapshot state_;
    UdpTransport transport_;
};
}  // namespace c2::fcs
