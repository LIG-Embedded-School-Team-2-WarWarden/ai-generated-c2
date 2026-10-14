#pragma once

#include "c2/protocol.hpp"
#include "c2/command_identity.hpp"

#include <cstdint>
#include <mutex>
#include <optional>
#include <variant>

namespace c2 {
struct AttackCommandConfig {
    std::uint64_t command_validity_us{};
    std::uint32_t first_command_id{1};
    std::uint32_t first_sequence{1};
};

enum class AttackStatusUpdateResult { stored, invalid, duplicate, out_of_order };

enum class AttackCommandError {
    unsupported_action,
    invalid_time,
    deadline_overflow,
    status_unavailable,
    target_unavailable,
    target_mismatch,
    invalid_state,
    not_aligned,
    not_armed,
    invalid_duration,
};

using AttackCommandResult = std::variant<AttackCommand, AttackCommandError>;

struct AttackCommandContext {
    std::optional<EffectorStatus> status;
    bool target_available{};
    std::uint64_t target_id{};
};

class AttackCommandService final {
public:
    explicit AttackCommandService(AttackCommandConfig config);

    [[nodiscard]] AttackStatusUpdateResult update_status(const EffectorStatus& status);
    [[nodiscard]] AttackCommandResult create(
        AttackAction action, std::uint64_t target_id,
        std::uint32_t duration_ms, std::uint64_t now_us);
    // Caller resolves session, connection and target freshness; this service
    // owns attack preconditions and command construction.
    [[nodiscard]] AttackCommandResult create(
        AttackAction action, std::uint64_t target_id, std::uint32_t duration_ms,
        std::uint64_t now_us, const AttackCommandContext& context);

private:
    [[nodiscard]] AttackCommandResult create_locked(
        AttackAction action, std::uint64_t target_id, std::uint32_t duration_ms,
        std::uint64_t now_us, const AttackCommandContext& context);
    AttackCommandConfig config_;
    std::mutex mutex_;
    std::optional<EffectorStatus> status_;
    CommandIdentity identity_;
};
}  // namespace c2
