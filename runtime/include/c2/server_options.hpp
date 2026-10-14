#pragma once

#include "c2/server_runtime.hpp"
#include "c2/datagram_processor.hpp"

#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <string_view>

namespace c2 {
struct ServerOptions {
    std::string bind_address{"0.0.0.0"};
    std::string observation_address{"127.0.0.1"};
    std::string effector_address{"127.0.0.1"};
    std::uint16_t observation_status_port{5001};
    std::uint16_t target_port{5002};
    std::uint16_t observation_command_port{5101};
    std::uint16_t effector_command_port{6001};
    std::uint16_t effector_status_port{6002};
    std::uint16_t asset_port{5000};
    DatagramProcessorConfig ingress;
    EventLogConfig event_log{"logs/c2-events.jsonl"};
    std::uint64_t target_validity_ms{2'000};
    std::size_t maximum_targets{256};
    std::size_t maximum_tracks_per_observer{64};
    std::size_t maximum_assets{256};
    std::uint64_t heartbeat_interval_ms{1'000};
    std::uint64_t heartbeat_timeout_ms{3'000};
    std::uint64_t retired_retention_ms{60'000};
    std::uint64_t maximum_registration_lease_ms{60'000};
    std::uint64_t status_timeout_ms{1'000};
    std::uint64_t command_validity_ms{500};
    std::uint64_t acknowledgement_timeout_ms{200};
    std::uint64_t completion_timeout_ms{2'000};
    std::uint32_t command_attempts{3};
    std::size_t maximum_pending_commands{1'024};
    std::size_t maximum_pending_per_asset{64};
    std::size_t maximum_command_outcomes{1'024};
    std::size_t maximum_assignments{256};
    std::size_t maximum_error_history{128};
    std::size_t maximum_inbound_rejections{128};
    AssignmentScoreWeights assignment_weights;
    bool auto_reassignment_enabled{true};
    std::uint32_t emergency_stop_repetitions{3};
};

[[nodiscard]] ServerOptions parse_server_options(
    std::span<const std::string_view> arguments);
[[nodiscard]] ServerRuntimeConfig make_server_runtime_config(
    const ServerOptions& options);
}  // namespace c2
