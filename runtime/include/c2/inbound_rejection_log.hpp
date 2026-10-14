#pragma once
#include "c2/runtime_types.hpp"
#include <deque>
#include <limits>
#include <mutex>
#include <stdexcept>

namespace c2 {
class InboundRejectionLog final {
public:
    explicit InboundRejectionLog(std::size_t capacity) : capacity_(capacity) {
        if (!capacity) throw std::invalid_argument("inbound rejection history must be non-zero");
    }
    void record(InboundRejectionCategory category, MessageKind kind,
                const MessageHeader* header, AssetRegistryResult reason, std::uint64_t time_us) {
        std::lock_guard lock(mutex_);
        events_.push_back({next_id_, category, kind, header ? header->asset_id : 0,
                          header ? header->session_id : 0, reason, time_us});
        next_id_ = next_id_ == std::numeric_limits<std::uint64_t>::max() ? 1 : next_id_ + 1;
        while (events_.size() > capacity_) events_.pop_front();
    }
    [[nodiscard]] std::vector<InboundRejection> snapshot() const {
        std::lock_guard lock(mutex_);
        return {events_.begin(), events_.end()};
    }
private:
    std::size_t capacity_;
    mutable std::mutex mutex_;
    std::deque<InboundRejection> events_;
    std::uint64_t next_id_{1};
};
} // namespace c2
