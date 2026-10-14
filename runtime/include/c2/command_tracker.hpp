#pragma once

#include "c2/protocol.hpp"
#include "c2/event_log.hpp"
#include "c2/udp_transport.hpp"

#include <cstddef>
#include <cstdint>
#include <deque>
#include <map>
#include <mutex>
#include <optional>
#include <vector>

namespace c2 {
struct CommandTrackerConfig {
    std::uint64_t delivery_ack_timeout_us{200'000};
    // Starts at the first progress ACK; subsequent progress does not extend it.
    std::uint64_t completion_timeout_us{2'000'000};
    std::uint32_t maximum_attempts{3};
    std::size_t maximum_pending{1'024};
    std::size_t maximum_pending_per_asset{64};
    std::size_t maximum_outcomes{1'024};
};

struct CommandKey {
    std::uint64_t asset_id{};
    std::uint64_t session_id{};
    std::uint32_t command_id{};
    auto operator<=>(const CommandKey&) const = default;
};

struct PendingCommand {
    std::uint64_t asset_id{};
    std::uint64_t session_id{};
    std::uint32_t command_id{};
    std::vector<std::byte> datagram;
    Endpoint endpoint;
    std::uint64_t sent_at_us{};
    // Delivery/acceptance deadline only, not a motion completion deadline.
    std::uint64_t valid_until_us{};
};

enum class CommandTrackResult {
    tracked,
    duplicate,
    invalid,
    capacity_exceeded,
    asset_capacity_exceeded,
};

enum class PendingCommandState { awaiting_delivery, awaiting_completion };
enum class AckUpdateResult { progress, terminal, duplicate, not_found, invalid };
enum class CommandTerminalState {
    completed,
    rejected,
    failed,
    expired,
    delivery_exhausted,
    completion_timeout,
    session_ended,
    cancelled_before_send,
};

enum class CommandDeliveryState { awaiting_send, sent, uncertain };

struct PendingCommandSnapshot {
    CommandKey key;
    PendingCommandState state{PendingCommandState::awaiting_delivery};
    std::uint32_t attempts{};
    std::uint64_t last_sent_at_us{};
    std::uint64_t completion_deadline_us{};
    CommandDeliveryState delivery{CommandDeliveryState::awaiting_send};
};

struct PendingTransmission {
    CommandKey key;
    std::vector<std::byte> datagram;
    Endpoint endpoint;
};

struct CommandOutcome {
    CommandKey key;
    CommandTerminalState state{CommandTerminalState::failed};
    std::uint64_t ended_at_us{};
};

struct CommandPollResult {
    std::vector<PendingTransmission> transmissions;
    std::vector<CommandOutcome> finalized;
};

class CommandTracker final {
public:
    explicit CommandTracker(CommandTrackerConfig config, EventSink events = {});

    [[nodiscard]] CommandTrackResult track(PendingCommand command);
    [[nodiscard]] AckUpdateResult observe(
        const CommandAck& acknowledgement, std::uint64_t received_at_us);
    [[nodiscard]] CommandPollResult poll(std::uint64_t now_us);
    [[nodiscard]] std::optional<PendingCommandSnapshot> pending(
        const CommandKey& key) const;
    [[nodiscard]] std::size_t pending_count() const;
    [[nodiscard]] std::vector<PendingCommandSnapshot> pending_commands() const;
    void record_send(const CommandKey& key, bool succeeded, std::uint64_t now_us);
    bool cancel_before_send(const CommandKey& key, std::uint64_t now_us);
    [[nodiscard]] std::vector<CommandOutcome> outcomes() const;
    std::size_t end_session(
        std::uint64_t asset_id, std::uint64_t session_id,
        std::uint64_t now_us);

private:
    struct Entry {
        PendingCommand command;
        PendingCommandState state{PendingCommandState::awaiting_delivery};
        std::uint32_t attempts{1};
        std::uint64_t last_sent_at_us{};
        std::uint64_t completion_deadline_us{};
        CommandDeliveryState delivery{CommandDeliveryState::awaiting_send};
    };

    void record_outcome_locked(const CommandOutcome& outcome);

    CommandTrackerConfig config_;
    EventSink events_;
    mutable std::mutex mutex_;
    std::map<CommandKey, Entry> pending_;
    std::deque<CommandOutcome> outcomes_;
};
}  // namespace c2
