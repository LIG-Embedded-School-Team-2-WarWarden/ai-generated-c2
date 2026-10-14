#pragma once
#include "c2/protocol.hpp"
#include <limits>
#include <mutex>
#include <stdexcept>

namespace c2 {
class CommandIdentity final {
public:
    CommandIdentity(std::uint32_t first_id, std::uint32_t first_sequence)
        : id_(first_id), sequence_(first_sequence) {
        if (!id_ || !sequence_) throw std::invalid_argument("command identity must be non-zero");
    }
    template <typename Command> void assign(Command& command) {
        std::lock_guard lock(mutex_);
        command.command_id = id_;
        command.header.sequence = sequence_;
        id_ = next(id_);
        sequence_ = next(sequence_);
    }
    static std::uint32_t next(std::uint32_t value) noexcept {
        return value == std::numeric_limits<std::uint32_t>::max() ? 1U : value + 1U;
    }
private:
    std::mutex mutex_;
    std::uint32_t id_;
    std::uint32_t sequence_;
};
} // namespace c2
