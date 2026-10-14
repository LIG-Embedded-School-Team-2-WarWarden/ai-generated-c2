#pragma once
#include "c2/udp_transport.hpp"
#include "c2/protocol.hpp"
#include <map>
#include <stdexcept>
#include <utility>

namespace c2 {
// Compatibility delivery tracking: any ACK completes a command, unlike the
// session-aware CommandTracker's delivery/completion state machine.
class LegacyCommandTracker final {
public:
    struct Transmission {
        std::vector<std::byte> datagram;
        Endpoint endpoint;
        std::uint32_t command_id{};
    };
    struct PollResult {
        std::vector<Transmission> transmissions;
        std::size_t exhausted{};
    };
    LegacyCommandTracker(std::uint64_t timeout_us, std::uint32_t maximum_attempts)
        : timeout_us_(timeout_us), maximum_attempts_(maximum_attempts) {
        if (timeout_us == 0 || maximum_attempts == 0)
            throw std::invalid_argument("command acknowledgement retry configuration is invalid");
    }
    void track(ComponentId source, std::uint32_t command_id,
               std::vector<std::byte> datagram, Endpoint endpoint, std::uint64_t now_us) {
        std::lock_guard lock(mutex_);
        pending_.insert_or_assign(key(source, command_id),
            Entry{{std::move(datagram), std::move(endpoint), command_id}, now_us, 1});
    }
    void acknowledge(const CommandAck& ack) {
        std::lock_guard lock(mutex_);
        pending_.erase(key(ack.header.source_id, ack.command_id));
    }
    [[nodiscard]] std::size_t pending_count() const {
        std::lock_guard lock(mutex_);
        return pending_.size();
    }
    [[nodiscard]] PollResult poll(std::uint64_t now_us) {
        std::lock_guard lock(mutex_);
        PollResult result;
        for (auto it = pending_.begin(); it != pending_.end();) {
            auto& entry = it->second;
            if (now_us < entry.last_sent_us || now_us - entry.last_sent_us < timeout_us_) {
                ++it;
            } else if (entry.attempts >= maximum_attempts_) {
                it = pending_.erase(it);
                ++result.exhausted;
            } else {
                entry.last_sent_us = now_us;
                ++entry.attempts;
                result.transmissions.push_back(entry.transmission);
                ++it;
            }
        }
        return result;
    }
private:
    static std::uint64_t key(ComponentId source, std::uint32_t id) noexcept {
        return (static_cast<std::uint64_t>(source) << 32U) | id;
    }
    struct Entry {
        Transmission transmission;
        std::uint64_t last_sent_us;
        std::uint32_t attempts;
    };
    std::uint64_t timeout_us_;
    std::uint32_t maximum_attempts_;
    mutable std::mutex mutex_;
    std::map<std::uint64_t, Entry> pending_;
};
} // namespace c2
