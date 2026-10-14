#pragma once

#include "c2/server_runtime.hpp"
#include "c2/udp_transport.hpp"
#include "c2/datagram_processor.hpp"

#include <cstdint>
#include <functional>

namespace c2 {
class ServerUdpIngress final {
public:
    using Clock = std::function<std::uint64_t()>;

    ServerUdpIngress(ServerRuntime& runtime, UdpConfig config, Clock clock,
                     DatagramProcessorConfig processor_config = {});
    ~ServerUdpIngress();
    void start();
    void stop() noexcept;
    [[nodiscard]] Endpoint local_endpoint() const;
    [[nodiscard]] bool running() const noexcept;
    [[nodiscard]] DatagramProcessorStats processing_stats() const;
    [[nodiscard]] UdpTransportStats transport_stats() const;

private:
    ServerRuntime& runtime_;
    Clock clock_;
    std::mutex lifecycle_mutex_;
    DatagramProcessor processor_;
    UdpTransport transport_;
};
}  // namespace c2
