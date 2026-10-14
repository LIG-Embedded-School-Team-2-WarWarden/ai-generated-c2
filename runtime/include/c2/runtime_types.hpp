#pragma once
#include "c2/asset_registry.hpp"
#include <variant>

namespace c2 {
enum class InboundRejectionCategory {
    invalid_packet,
    unsupported_message,
    registration,
    authentication,
    state_update,
};

struct InboundRejection {
    std::uint64_t event_id{};
    InboundRejectionCategory category{InboundRejectionCategory::invalid_packet};
    MessageKind message_kind{MessageKind::unspecified};
    std::uint64_t asset_id{};
    std::uint64_t session_id{};
    AssetRegistryResult reason{AssetRegistryResult::invalid};
    std::uint64_t occurred_at_us{};
};

struct CommandRetryResult {
    std::size_t resent{};
    std::size_t exhausted{};
    std::size_t send_failed{};
};

struct EmergencyStopResult {
    std::size_t assets{};
    std::size_t datagrams{};
};

enum class InboundResult { accepted, invalid_packet, unsupported_message, rejected };
enum class DispatchError {
    connection_unavailable,
    pose_resynchronization_required,
    command_rejected,
    delivery_uncertain,
};

using ObservationDispatchResult =
    std::variant<ObservationTurretCommand, DispatchError>;
using EffectorDispatchResult = std::variant<EffectorTurretCommand, DispatchError>;
using AttackDispatchResult = std::variant<AttackCommand, DispatchError>;
using DevelopmentPoseDispatchResult = std::variant<DevelopmentPoseCommand, DispatchError>;

} // namespace c2
