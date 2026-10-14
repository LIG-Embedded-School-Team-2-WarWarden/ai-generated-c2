#include "c2/telemetry_store.hpp"
#include "c2/protocol_validation.hpp"
#include <stdexcept>

namespace c2 {
TelemetryStore::TelemetryStore(const std::size_t maximum_errors)
    : maximum_errors_(maximum_errors) {
    if (maximum_errors_ == 0) throw std::invalid_argument("maximum errors must be non-zero");
}

TelemetryUpdateResult TelemetryStore::update(const ObservationStatus& value) {
    if (!validate(value).valid()) return TelemetryUpdateResult::invalid;
    std::lock_guard lock(mutex_);
    if (observation_) {
        if (value.header.sequence == observation_->header.sequence && value.timestamp_us == observation_->timestamp_us)
            return TelemetryUpdateResult::duplicate;
        if (value.timestamp_us <= observation_->timestamp_us) return TelemetryUpdateResult::out_of_order;
    }
    observation_ = value;
    return TelemetryUpdateResult::stored;
}

TelemetryUpdateResult TelemetryStore::update(const EffectorStatus& value) {
    if (!validate(value).valid()) return TelemetryUpdateResult::invalid;
    std::lock_guard lock(mutex_);
    if (effector_) {
        if (value.header.sequence == effector_->header.sequence && value.timestamp_us == effector_->timestamp_us)
            return TelemetryUpdateResult::duplicate;
        if (value.timestamp_us <= effector_->timestamp_us) return TelemetryUpdateResult::out_of_order;
    }
    effector_ = value;
    return TelemetryUpdateResult::stored;
}

TelemetryUpdateResult TelemetryStore::update(const CommandAck& value) {
    if (!validate(value).valid()) return TelemetryUpdateResult::invalid;
    std::lock_guard lock(mutex_);
    auto& current = acknowledgements_[ack_key(value.header.source_id, value.command_id)];
    if (current.command_id != 0) {
        if (value.result == current.result && value.timestamp_us == current.timestamp_us)
            return TelemetryUpdateResult::duplicate;
        if (value.timestamp_us <= current.timestamp_us ||
            static_cast<std::uint32_t>(value.result) < static_cast<std::uint32_t>(current.result))
            return TelemetryUpdateResult::out_of_order;
    }
    current = value;
    return TelemetryUpdateResult::stored;
}

TelemetryUpdateResult TelemetryStore::update(const ErrorReport& value) {
    if (!validate(value).valid()) return TelemetryUpdateResult::invalid;
    std::lock_guard lock(mutex_);
    if (!errors_.empty() && value.header.source_id == errors_.back().header.source_id &&
        value.header.sequence == errors_.back().header.sequence && value.timestamp_us == errors_.back().timestamp_us)
        return TelemetryUpdateResult::duplicate;
    errors_.push_back(value);
    while (errors_.size() > maximum_errors_) errors_.pop_front();
    return TelemetryUpdateResult::stored;
}

std::optional<ObservationStatus> TelemetryStore::observation_status() const {
    std::lock_guard lock(mutex_); return observation_;
}
std::optional<EffectorStatus> TelemetryStore::effector_status() const {
    std::lock_guard lock(mutex_); return effector_;
}
std::optional<CommandAck> TelemetryStore::acknowledgement(ComponentId source, std::uint32_t command_id) const {
    std::lock_guard lock(mutex_); const auto found = acknowledgements_.find(ack_key(source, command_id));
    return found == acknowledgements_.end() ? std::nullopt : std::optional<CommandAck>{found->second};
}
std::vector<ErrorReport> TelemetryStore::errors() const {
    std::lock_guard lock(mutex_); return {errors_.begin(), errors_.end()};
}
std::uint64_t TelemetryStore::ack_key(ComponentId source, std::uint32_t command_id) noexcept {
    return (static_cast<std::uint64_t>(source) << 32U) | command_id;
}
}  // namespace c2
