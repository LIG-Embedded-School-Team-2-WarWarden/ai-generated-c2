#include "c2/command_tracker.hpp"

#include "c2/protocol_validation.hpp"

#include <limits>
#include <stdexcept>

namespace c2 {
namespace {
std::uint64_t deadline_after(
    const std::uint64_t now_us, const std::uint64_t duration_us) noexcept {
    if (duration_us > std::numeric_limits<std::uint64_t>::max() - now_us)
        return std::numeric_limits<std::uint64_t>::max();
    return now_us + duration_us;
}

bool is_progress(const CommandResult result) noexcept {
    return result == CommandResult::received || result == CommandResult::accepted ||
           result == CommandResult::in_progress;
}

std::optional<CommandTerminalState> terminal_state(
    const CommandResult result) noexcept {
    switch (result) {
        case CommandResult::completed: return CommandTerminalState::completed;
        case CommandResult::rejected: return CommandTerminalState::rejected;
        case CommandResult::failed: return CommandTerminalState::failed;
        default: return std::nullopt;
    }
}
}

CommandTracker::CommandTracker(const CommandTrackerConfig config, EventSink events)
    : config_(config), events_(std::move(events)) {
    if (config_.delivery_ack_timeout_us == 0 ||
        config_.completion_timeout_us == 0 || config_.maximum_attempts == 0 ||
        config_.maximum_pending == 0 ||
        config_.maximum_pending_per_asset == 0 ||
        config_.maximum_pending_per_asset > config_.maximum_pending ||
        config_.maximum_outcomes == 0)
        throw std::invalid_argument("command tracker configuration is invalid");
}

CommandTrackResult CommandTracker::track(PendingCommand command) {
    if (command.asset_id == 0 || command.session_id == 0 ||
        command.command_id == 0 || command.datagram.empty() ||
        command.endpoint.address.empty() || command.endpoint.port == 0 ||
        command.sent_at_us == 0 ||
        command.valid_until_us <= command.sent_at_us)
        return CommandTrackResult::invalid;
    const CommandKey key{command.asset_id, command.session_id, command.command_id};
    std::unique_lock lock(mutex_);
    if (pending_.contains(key)) return CommandTrackResult::duplicate;
    if (pending_.size() >= config_.maximum_pending)
        return CommandTrackResult::capacity_exceeded;
    std::size_t asset_count{};
    for (const auto& [pending_key, entry] : pending_) {
        (void)entry;
        if (pending_key.asset_id == command.asset_id) ++asset_count;
    }
    if (asset_count >= config_.maximum_pending_per_asset)
        return CommandTrackResult::asset_capacity_exceeded;
    Entry entry;
    entry.last_sent_at_us = command.sent_at_us;
    entry.command = std::move(command);
    const auto prepared_at = entry.command.sent_at_us;
    pending_.emplace(key, std::move(entry));
    lock.unlock();
    emit_event(events_, {"command_prepared", prepared_at, key.asset_id, key.session_id, key.command_id, ""});
    return CommandTrackResult::tracked;
}

AckUpdateResult CommandTracker::observe(
    const CommandAck& acknowledgement, const std::uint64_t received_at_us) {
    if (!validate(acknowledgement).valid() || received_at_us == 0)
        return AckUpdateResult::invalid;
    const CommandKey key{acknowledgement.header.asset_id,
                         acknowledgement.header.session_id,
                         acknowledgement.command_id};
    std::optional<CommandOutcome> emitted;
    const auto result = [&]() -> AckUpdateResult {
        std::lock_guard lock(mutex_);
        const auto found = pending_.find(key);
        if (found == pending_.end()) return AckUpdateResult::not_found;
        const auto& current = found->second;
        const bool delivery_expired =
            current.state == PendingCommandState::awaiting_delivery &&
            received_at_us >= current.command.valid_until_us;
        const bool completion_expired =
            current.state == PendingCommandState::awaiting_completion &&
            received_at_us >= current.completion_deadline_us;
        if (delivery_expired || completion_expired) {
            emitted = CommandOutcome{key, delivery_expired ? CommandTerminalState::expired
                                                            : CommandTerminalState::completion_timeout, received_at_us};
            record_outcome_locked(*emitted);
            pending_.erase(found);
            return AckUpdateResult::not_found;
        }
        if (const auto terminal = terminal_state(acknowledgement.result)) {
            emitted = CommandOutcome{key, *terminal, received_at_us};
            record_outcome_locked(*emitted);
            pending_.erase(found);
            return AckUpdateResult::terminal;
        }
        if (!is_progress(acknowledgement.result)) return AckUpdateResult::invalid;
        auto& entry = found->second;
        if (entry.state == PendingCommandState::awaiting_completion)
            return AckUpdateResult::progress;
        entry.state = PendingCommandState::awaiting_completion;
        entry.completion_deadline_us = deadline_after(
            received_at_us, config_.completion_timeout_us);
        return AckUpdateResult::progress;
    }();
    if (emitted) emit_event(events_, {"command_outcome", emitted->ended_at_us, key.asset_id,
        key.session_id, key.command_id, std::to_string(static_cast<int>(emitted->state))});
    return result;

}

CommandPollResult CommandTracker::poll(const std::uint64_t now_us) {
    CommandPollResult result;
    std::unique_lock lock(mutex_);
    for (auto iterator = pending_.begin(); iterator != pending_.end();) {
        auto& entry = iterator->second;
        std::optional<CommandTerminalState> terminal;
        if (entry.state == PendingCommandState::awaiting_delivery &&
            now_us >= entry.command.valid_until_us) {
            terminal = CommandTerminalState::expired;
        } else if (entry.state == PendingCommandState::awaiting_completion) {
            if (now_us >= entry.completion_deadline_us)
                terminal = CommandTerminalState::completion_timeout;
        } else if (now_us >= entry.last_sent_at_us &&
                   now_us - entry.last_sent_at_us >=
                       config_.delivery_ack_timeout_us) {
            if (entry.attempts >= config_.maximum_attempts) {
                terminal = CommandTerminalState::delivery_exhausted;
            } else {
                ++entry.attempts;
                entry.last_sent_at_us = now_us;
                result.transmissions.push_back(
                    {iterator->first, entry.command.datagram,
                     entry.command.endpoint});
            }
        }
        if (terminal) {
            CommandOutcome outcome{iterator->first, *terminal, now_us};
            record_outcome_locked(outcome);
            result.finalized.push_back(outcome);
            iterator = pending_.erase(iterator);
        } else {
            ++iterator;
        }
    }
    lock.unlock();
    for (const auto& outcome : result.finalized)
        emit_event(events_, {"command_outcome", outcome.ended_at_us, outcome.key.asset_id,
            outcome.key.session_id, outcome.key.command_id, std::to_string(static_cast<int>(outcome.state))});
    return result;
}

std::optional<PendingCommandSnapshot> CommandTracker::pending(
    const CommandKey& key) const {
    std::lock_guard lock(mutex_);
    const auto found = pending_.find(key);
    if (found == pending_.end()) return std::nullopt;
    const auto& entry = found->second;
    return PendingCommandSnapshot{key, entry.state, entry.attempts,
                                  entry.last_sent_at_us,
                                  entry.completion_deadline_us, entry.delivery};
}

std::vector<PendingCommandSnapshot> CommandTracker::pending_commands() const {
    std::lock_guard lock(mutex_);
    std::vector<PendingCommandSnapshot> result;
    for (const auto& [key, entry] : pending_)
        result.push_back({key, entry.state, entry.attempts, entry.last_sent_at_us,
                          entry.completion_deadline_us, entry.delivery});
    return result;
}
void CommandTracker::record_send(const CommandKey& key, bool succeeded, std::uint64_t now_us) {
    std::unique_lock lock(mutex_);
    const auto it = pending_.find(key);
    if (it != pending_.end())
        it->second.delivery = succeeded ? CommandDeliveryState::sent : CommandDeliveryState::uncertain;
    lock.unlock();
    emit_event(events_, {succeeded ? "command_send_succeeded" : "command_delivery_uncertain",
        now_us, key.asset_id, key.session_id, key.command_id, ""});
}
bool CommandTracker::cancel_before_send(const CommandKey& key, std::uint64_t now_us) {
    std::unique_lock lock(mutex_);
    const auto it = pending_.find(key);
    if (it == pending_.end() || it->second.delivery != CommandDeliveryState::awaiting_send) return false;
    record_outcome_locked({key, CommandTerminalState::cancelled_before_send, now_us});
    pending_.erase(it);
    lock.unlock();
    emit_event(events_, {"command_outcome", now_us, key.asset_id, key.session_id, key.command_id,
        std::to_string(static_cast<int>(CommandTerminalState::cancelled_before_send))});
    return true;
}

std::size_t CommandTracker::pending_count() const {
    std::lock_guard lock(mutex_);
    return pending_.size();
}

std::vector<CommandOutcome> CommandTracker::outcomes() const {
    std::lock_guard lock(mutex_);
    return {outcomes_.begin(), outcomes_.end()};
}

std::size_t CommandTracker::end_session(
    const std::uint64_t asset_id, const std::uint64_t session_id,
    const std::uint64_t now_us) {
    std::unique_lock lock(mutex_);
    std::vector<CommandOutcome> emitted;
    std::size_t ended{};
    for (auto iterator = pending_.begin(); iterator != pending_.end();) {
        if (iterator->first.asset_id == asset_id &&
            iterator->first.session_id == session_id) {
            const CommandOutcome outcome{iterator->first, CommandTerminalState::session_ended, now_us};
            record_outcome_locked(outcome);
            emitted.push_back(outcome);
            iterator = pending_.erase(iterator);
            ++ended;
        } else {
            ++iterator;
        }
    }
    lock.unlock();
    for (const auto& outcome : emitted)
        emit_event(events_, {"command_outcome", now_us, outcome.key.asset_id, outcome.key.session_id,
            outcome.key.command_id, std::to_string(static_cast<int>(outcome.state))});
    return ended;
}

void CommandTracker::record_outcome_locked(const CommandOutcome& outcome) {
    if (outcomes_.size() >= config_.maximum_outcomes) outcomes_.pop_front();
    outcomes_.push_back(outcome);
}
}  // namespace c2
