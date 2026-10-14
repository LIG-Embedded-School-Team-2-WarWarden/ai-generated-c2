#include "c2/dummy_assets.hpp"
#include "c2/protobuf_codec.hpp"
#include "c2/udp_transport.hpp"

#include <chrono>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <random>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <variant>

namespace {
struct Options {
    std::string bind_address{"0.0.0.0"};
    std::string c2_address{"127.0.0.1"};
    std::uint16_t listen_port{5101};
    std::uint16_t status_port{5001};
    std::uint16_t target_port{5002};
    std::uint16_t c2_port{5000};
    std::uint64_t asset_id{};
    std::uint64_t capabilities{c2::capability::observation_scan | c2::capability::development_pose};
    std::uint64_t registration_interval_ms{1'000};
    std::uint64_t lease_ms{5'000};
    float x_m{};
    float y_m{};
    float z_m{1.5F};
    float azimuth_deg{};
    // Precomputed PROJECT_FRAME target data. The dummy does not interpret
    // LiDAR-local coordinates or apply sensor/mount calibration.
    float target_x_m{100.0F};
    float target_y_m{20.0F};
    float target_z_m{10.0F};
    float vx_mps{};
    float vy_mps{};
    float vz_mps{};
    bool velocity_valid{true};
    c2::ObservationTurretLimits limits{-180, 180, -90, 90};
    std::uint64_t status_interval_ms{100};
    std::uint64_t heartbeat_interval_ms{1'000};
    std::uint64_t target_interval_ms{1'000};
    std::uint64_t watchdog_timeout_ms{3'000};
};

template <typename Value>
Value positive_number(const std::string_view text, const char* name) {
    Value value{};
    const auto result = std::from_chars(text.data(), text.data() + text.size(), value);
    if (result.ec != std::errc{} || result.ptr != text.data() + text.size() || value == 0)
        throw std::invalid_argument(std::string{"invalid "} + name);
    return value;
}

std::uint64_t interval_ms(const std::string_view text, const char* name) {
    const auto value = positive_number<std::uint64_t>(text, name);
    if (value > static_cast<std::uint64_t>(
                    std::numeric_limits<std::chrono::milliseconds::rep>::max()))
        throw std::invalid_argument(std::string{name} + " is too large");
    return value;
}

std::uint16_t port(const std::string_view value, const bool allow_zero = false) {
    unsigned parsed{};
    const auto result = std::from_chars(
        value.data(), value.data() + value.size(), parsed);
    if (result.ec != std::errc{} || result.ptr != value.data() + value.size() ||
        (!allow_zero && parsed == 0) || parsed > std::numeric_limits<std::uint16_t>::max())
        throw std::invalid_argument("invalid UDP port");
    return static_cast<std::uint16_t>(parsed);
}

float finite_float(const std::string& value, const char* name) {
    std::size_t used{};
    const auto parsed = std::stof(value, &used);
    if (used != value.size() || !std::isfinite(parsed))
        throw std::invalid_argument(std::string{"invalid "} + name);
    return parsed;
}

Options parse_options(const int argc, char* argv[]) {
    Options options;
    for (int index = 1; index < argc; index += 2) {
        if (index + 1 >= argc) throw std::invalid_argument("option value is missing");
        const std::string_view name{argv[index]};
        const std::string value{argv[index + 1]};
        if (name == "--bind") options.bind_address = value;
        else if (name == "--listen-port") options.listen_port = port(value, true);
        else if (name == "--asset-id") options.asset_id = positive_number<std::uint64_t>(value, "asset id");
        else if (name == "--c2-ip") options.c2_address = value;
        else if (name == "--c2-port") options.c2_port = port(value);
        else if (name == "--capabilities") options.capabilities = positive_number<std::uint64_t>(value, "capabilities");
        else if (name == "--registration-interval-ms") options.registration_interval_ms = interval_ms(value, "registration interval");
        else if (name == "--lease-ms") options.lease_ms = interval_ms(value, "registration lease");
        else if (name == "--x") options.x_m = finite_float(value, "x");
        else if (name == "--y") options.y_m = finite_float(value, "y");
        else if (name == "--z") options.z_m = finite_float(value, "z");
        else if (name == "--azimuth") options.azimuth_deg = finite_float(value, "azimuth");
        else if (name == "--target-x") options.target_x_m = finite_float(value, "target x");
        else if (name == "--target-y") options.target_y_m = finite_float(value, "target y");
        else if (name == "--target-z") options.target_z_m = finite_float(value, "target z");
        else if (name == "--vx") options.vx_mps = finite_float(value, "vx");
        else if (name == "--vy") options.vy_mps = finite_float(value, "vy");
        else if (name == "--vz") options.vz_mps = finite_float(value, "vz");
        else if (name == "--velocity-valid") {
            if (value != "0" && value != "1")
                throw std::invalid_argument("velocity-valid must be 0 or 1");
            options.velocity_valid = value == "1";
        }
        else if (name == "--min-pan") options.limits.minimum_pan_deg = finite_float(value, "minimum pan");
        else if (name == "--max-pan") options.limits.maximum_pan_deg = finite_float(value, "maximum pan");
        else if (name == "--min-tilt") options.limits.minimum_tilt_deg = finite_float(value, "minimum tilt");
        else if (name == "--max-tilt") options.limits.maximum_tilt_deg = finite_float(value, "maximum tilt");
        else if (name == "--status-port") options.status_port = port(value);
        else if (name == "--target-port") options.target_port = port(value);
        else if (name == "--status-interval-ms")
            options.status_interval_ms = interval_ms(value, "status interval");
        else if (name == "--heartbeat-interval-ms")
            options.heartbeat_interval_ms = interval_ms(value, "heartbeat interval");
        else if (name == "--target-interval-ms")
            options.target_interval_ms = interval_ms(value, "target interval");
        else if (name == "--watchdog-timeout-ms")
            options.watchdog_timeout_ms = positive_number<std::uint64_t>(value, "watchdog timeout");
        else throw std::invalid_argument("unknown option: " + std::string{name});
    }
    if (options.watchdog_timeout_ms > std::numeric_limits<std::uint64_t>::max() / 1000U)
        throw std::invalid_argument("watchdog timeout is too large");
    if (options.lease_ms > std::numeric_limits<std::uint64_t>::max() / 1000U ||
        options.registration_interval_ms >= options.lease_ms)
        throw std::invalid_argument("registration interval must be shorter than lease");
    return options;
}

std::uint64_t now_us() {
    return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count());
}

std::uint64_t session_id() {
    std::random_device source;
    const auto random = (static_cast<std::uint64_t>(source()) << 32U) | source();
    const auto value = random ^ now_us();
    return value == 0 ? 1 : value;
}

template <typename Message>
void send(c2::UdpTransport& transport, const Message& message, const c2::Endpoint& endpoint) {
    const auto bytes = c2::protobuf::encode(c2::Envelope{message});
    transport.send(bytes, endpoint);
}
}  // namespace

int main(int argc, char* argv[]) {
    try {
        const auto options = parse_options(argc, argv);
        const bool dynamic = options.asset_id != 0;
        const auto session = dynamic ? session_id() : 1U;
        const c2::Endpoint status_endpoint{
            options.c2_address, dynamic ? options.c2_port : options.status_port};
        const c2::Endpoint target_endpoint{
            options.c2_address, dynamic ? options.c2_port : options.target_port};
        c2::DummyObservationAsset asset(
            {{c2::protocol_version, 1, now_us(), c2::ComponentId::observation_asset,
              c2::ComponentId::command_and_control,
              dynamic ? options.asset_id : 1U, session},
             c2::CoordinateFrame::project_frame, options.x_m, options.y_m,
             options.z_m, options.azimuth_deg},
            options.limits);
        c2::UdpTransport* transport_ptr{};
        c2::UdpTransport transport(
            {options.bind_address, options.listen_port},
            [&](std::vector<std::byte> data, c2::Endpoint) {
            const auto decoded = c2::protobuf::decode(data);
            if (!std::holds_alternative<c2::Envelope>(decoded)) return;
            const auto& payload = std::get<c2::Envelope>(decoded).payload;
            const auto received = now_us();
            if (const auto* heartbeat = std::get_if<c2::Heartbeat>(&payload)) {
                (void)asset.observe_control_heartbeat(*heartbeat, received);
            } else if (const auto* command = std::get_if<c2::DevelopmentPoseCommand>(&payload)) {
                if ((options.capabilities & c2::capability::development_pose) == 0) return;
                const auto result = asset.handle(*command, received);
                send(*transport_ptr, result.acknowledgement, status_endpoint);
                if (result.error_report)
                    send(*transport_ptr, *result.error_report, status_endpoint);
                send(*transport_ptr, asset.asset_pose(received), status_endpoint);
            } else if (const auto* command = std::get_if<c2::ObservationTurretCommand>(&payload)) {
                const auto result = asset.handle(*command, received);
                send(*transport_ptr, result.acknowledgement, status_endpoint);
                if (result.error_report)
                    send(*transport_ptr, *result.error_report, status_endpoint);
                send(*transport_ptr, asset.status(received), status_endpoint);
            }
            });
        transport_ptr = &transport;
        transport.start();
        const auto command_port = transport.local_endpoint().port;
        const c2::AssetRegistration registration{
            {c2::protocol_version, 1, now_us(), c2::ComponentId::observation_asset,
             c2::ComponentId::command_and_control, options.asset_id, session},
            c2::AssetRole::observation, command_port, options.capabilities,
            "dummy-observation/2.0", {},
            {options.limits.minimum_pan_deg, options.limits.maximum_pan_deg,
             options.limits.minimum_tilt_deg, options.limits.maximum_tilt_deg},
            false, options.lease_ms};
        const auto started = std::chrono::steady_clock::now();
        std::jthread publisher([&](const std::stop_token stop) {
            auto next_status = std::chrono::steady_clock::now();
            auto next_heartbeat = next_status;
            auto next_target = next_status;
            auto next_registration = next_status;
            while (!stop.stop_requested()) {
                const auto now = now_us();
                const auto steady_now = std::chrono::steady_clock::now();
                const auto uptime = static_cast<std::uint64_t>(
                    std::chrono::duration_cast<std::chrono::milliseconds>(
                        steady_now - started).count());
                if (dynamic && steady_now >= next_registration) {
                    send(transport, registration, status_endpoint);
                    next_registration = steady_now + std::chrono::milliseconds(
                        options.registration_interval_ms);
                }
                if (const auto report = asset.check_watchdog(
                        now, options.watchdog_timeout_ms * 1000U))
                    send(transport, *report, status_endpoint);
                if (steady_now >= next_status) {
                    send(transport, asset.status(now), status_endpoint);
                    next_status = steady_now +
                        std::chrono::milliseconds(options.status_interval_ms);
                }
                if (steady_now >= next_heartbeat) {
                    send(transport, asset.asset_pose(now), status_endpoint);
                    send(transport, asset.heartbeat(now, uptime), status_endpoint);
                    next_heartbeat = steady_now +
                        std::chrono::milliseconds(options.heartbeat_interval_ms);
                }
                if (steady_now >= next_target) {
                    send(transport, asset.target(
                        1, options.target_x_m, options.target_y_m, options.target_z_m,
                        options.vx_mps, options.vy_mps, options.vz_mps,
                        options.velocity_valid, 0.95F, now), target_endpoint);
                    next_target = steady_now +
                        std::chrono::milliseconds(options.target_interval_ms);
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        });
        std::cout << "Dummy observation asset started on UDP " << options.listen_port
                  << ". Type quit to stop.\n";
        std::string line;
        while (std::getline(std::cin, line) && line != "quit") {}
        publisher.request_stop();
        publisher.join();
        if (dynamic) {
            c2::AssetUnregister unregister{
                {c2::protocol_version, 2, now_us(),
                 c2::ComponentId::observation_asset,
                 c2::ComponentId::command_and_control,
                 options.asset_id, session}, "normal shutdown"};
            send(transport, unregister, status_endpoint);
        }
        transport.stop();
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Dummy observation asset error: " << error.what() << '\n';
        return 1;
    }
}
