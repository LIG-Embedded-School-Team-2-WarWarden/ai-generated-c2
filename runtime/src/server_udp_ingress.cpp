#include "c2/server_udp_ingress.hpp"

#include <stdexcept>
#include <utility>

namespace c2 {
ServerUdpIngress::ServerUdpIngress(
    ServerRuntime& runtime, UdpConfig config, Clock clock,
    DatagramProcessorConfig processor_config)
    : runtime_(runtime),
      clock_(std::move(clock)),
      processor_(processor_config, [this](InboundDatagram datagram) {
          (void)runtime_.ingest(datagram.bytes, datagram.source, datagram.received_at_us);
      }),
      transport_(std::move(config),
                 [this](std::vector<std::byte> data, Endpoint source) {
                     const auto received_at_us = clock_();
                     if (received_at_us != 0)
                         (void)processor_.submit({std::move(data), std::move(source), received_at_us});
                 }) {
    if (!clock_) throw std::invalid_argument("server ingress clock must be set");
}

ServerUdpIngress::~ServerUdpIngress() { stop(); }
void ServerUdpIngress::start() {
    std::lock_guard lock(lifecycle_mutex_);
    processor_.start();
    try { transport_.start(); }
    catch (...) { processor_.stop(); throw; }
}
void ServerUdpIngress::stop() noexcept {
    std::lock_guard lock(lifecycle_mutex_);
    transport_.stop();
    processor_.stop();
}
DatagramProcessorStats ServerUdpIngress::processing_stats() const { return processor_.stats(); }
UdpTransportStats ServerUdpIngress::transport_stats() const { return transport_.stats(); }
Endpoint ServerUdpIngress::local_endpoint() const { return transport_.local_endpoint(); }
bool ServerUdpIngress::running() const noexcept { return transport_.running(); }
}  // namespace c2
