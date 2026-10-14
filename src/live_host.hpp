#pragma once
#include "c2/server_options.hpp"
#include "c2/server_udp_ingress.hpp"
#include <atomic>
#include <chrono>
#include <memory>
#include <thread>

namespace ai_c2 {
inline std::uint64_t now_us() {
    return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count());
}
// Owns real sockets and workers. Destroy ingress and workers before the runtime.
class LiveHost {
    c2::UdpTransport sender_;
    c2::EventLog log_;
    std::unique_ptr<c2::ServerRuntime> runtime_;
    std::unique_ptr<c2::ServerUdpIngress> ingress_;
    std::jthread worker_;
    std::atomic<std::uint64_t> worker_errors_{};
public:
    explicit LiveHost(c2::ServerOptions options)
        : sender_({options.bind_address, 0}, [](auto, auto) {}), log_(options.event_log) {
        sender_.start();
        auto config = c2::make_server_runtime_config(options);
        if (!options.event_log.path.empty()) config.event_sink = [this](const auto& e) { log_.append(e); };
        runtime_ = std::make_unique<c2::ServerRuntime>(config,
            [this](auto bytes, const auto& endpoint) { sender_.send(bytes, endpoint); });
        ingress_ = std::make_unique<c2::ServerUdpIngress>(*runtime_,
            c2::UdpConfig{options.bind_address, options.asset_port}, now_us, options.ingress);
        ingress_->start();
        worker_ = std::jthread([this, interval = options.heartbeat_interval_ms](std::stop_token stop) {
            const auto started = std::chrono::steady_clock::now();
            auto next_heartbeat = started;
            while (!stop.stop_requested()) {
                try {
                    const auto steady = std::chrono::steady_clock::now();
                    const auto now = now_us();
                    if (steady >= next_heartbeat) {
                        runtime_->send_heartbeats(now, static_cast<std::uint64_t>(
                            std::chrono::duration_cast<std::chrono::milliseconds>(steady - started).count()));
                        next_heartbeat = steady + std::chrono::milliseconds(interval);
                    }
                    (void)runtime_->retry_unacknowledged(now);
                } catch (...) { ++worker_errors_; }
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }
        });
    }
    ~LiveHost() {
        worker_.request_stop();
        if (worker_.joinable()) worker_.join();
        if (ingress_) ingress_->stop();
        sender_.stop();
    }
    c2::ServerRuntime& runtime() { return *runtime_; }
    c2::UdpTransportStats stats() const { return ingress_->transport_stats(); }
    c2::Endpoint endpoint() const { return ingress_->local_endpoint(); }
    std::uint64_t worker_errors() const { return worker_errors_; }
};
}
