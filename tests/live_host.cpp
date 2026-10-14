#include "live_host.hpp"
#include "c2/dummy_assets.hpp"
#include "c2/protobuf_codec.hpp"
#include <iostream>
#include <stdexcept>

template<class Predicate> void wait_for(Predicate predicate, const char* message) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (!predicate()) {
        if (std::chrono::steady_clock::now() >= deadline) throw std::runtime_error(message);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}
int main() {
    c2::ServerOptions options;
    options.bind_address = "127.0.0.1"; options.asset_port = 0;
    options.event_log.path.clear(); options.heartbeat_interval_ms = 50;
    ai_c2::LiveHost host(options);
    auto& runtime = host.runtime();
    const auto endpoint = host.endpoint();
    c2::DummyObservationAsset observation(
        {{3, 1, ai_c2::now_us(), c2::ComponentId::observation_asset,
          c2::ComponentId::command_and_control, 101, 123},
          c2::CoordinateFrame::project_frame, 0, 0, 1.5F, 0}, {-180, 180, -90, 90});
    std::atomic<int> commands{}, heartbeats{};
    c2::UdpTransport* peer_ptr{};
    c2::UdpTransport peer({"127.0.0.1", 0}, [&](auto bytes, auto) {
        const auto decoded = c2::protobuf::decode(bytes);
        if (!std::holds_alternative<c2::Envelope>(decoded)) return;
        const auto& payload = std::get<c2::Envelope>(decoded).payload;
        if (std::holds_alternative<c2::Heartbeat>(payload)) ++heartbeats;
        if (const auto* command = std::get_if<c2::ObservationTurretCommand>(&payload)) {
            // Intentionally lose the first packet to verify the real retry worker.
            if (++commands == 1) return;
            const auto reply = observation.handle(*command, ai_c2::now_us());
            peer_ptr->send(c2::protobuf::encode(c2::Envelope{reply.acknowledgement}), endpoint);
        }
    });
    peer_ptr = &peer; peer.start();
    const auto send = [&](const auto& message) { peer.send(c2::protobuf::encode(c2::Envelope{message}), endpoint); };
    send(c2::AssetRegistration{
        {3, 1, ai_c2::now_us(), c2::ComponentId::observation_asset,
         c2::ComponentId::command_and_control, 101, 123},
        c2::AssetRole::observation, peer.local_endpoint().port, c2::capability::observation_scan,
        "integration-test", {}, {-180, 180, -90, 90}, false, 5000});
    send(observation.heartbeat(ai_c2::now_us(), 1));
    send(observation.asset_pose(ai_c2::now_us()));
    send(observation.status(ai_c2::now_us()));
    send(observation.target(1, 10, 20, 2, 0.9F, ai_c2::now_us()));
    wait_for([&] {
        auto assets = runtime.assets(ai_c2::now_us());
        return assets.size() == 1 && assets[0].pose_synchronized &&
            assets[0].connection_state == c2::AssetConnectionState::connected &&
            runtime.tracks(ai_c2::now_us()).size() == 1;
    }, "registration/heartbeat/pose/target not received over UDP");
    wait_for([&] { return heartbeats.load() != 0; }, "C2 heartbeat not received by asset");
    const auto command = runtime.command_observation(101, c2::ObservationTurretCommandType::scan, 10, 5, ai_c2::now_us());
    if (!std::holds_alternative<c2::ObservationTurretCommand>(command)) throw std::runtime_error("scan rejected");
    wait_for([&] {
        const auto outcomes = runtime.command_outcomes();
        return !outcomes.empty() && outcomes.back().state == c2::CommandTerminalState::completed;
    }, "UDP retransmission/ACK completion failed");
    if (commands < 2 || runtime.pending_command_count() != 0) throw std::runtime_error("retry or ACK cleanup failed");
    peer.send(std::vector<std::byte>{std::byte{0xff}}, endpoint);
    wait_for([&] { return !runtime.inbound_rejections().empty(); }, "malformed packet not audited");
    peer.stop();
    if (host.worker_errors()) throw std::runtime_error("worker exception");
    std::cout << "Live dashboard host UDP passed: registration, pose, target, bidirectional heartbeat, packet loss/retry, terminal ACK, malformed rejection\n";
}
