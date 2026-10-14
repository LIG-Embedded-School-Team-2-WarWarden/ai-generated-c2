#pragma once
#include "c2/protocol.hpp"
#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <optional>
#include <unordered_map>
#include <vector>

namespace c2 {
enum class TelemetryUpdateResult { stored, invalid, duplicate, out_of_order };
class TelemetryStore final {
public:
    explicit TelemetryStore(std::size_t maximum_errors = 128);
    [[nodiscard]] TelemetryUpdateResult update(const ObservationStatus& value);
    [[nodiscard]] TelemetryUpdateResult update(const EffectorStatus& value);
    [[nodiscard]] TelemetryUpdateResult update(const CommandAck& value);
    [[nodiscard]] TelemetryUpdateResult update(const ErrorReport& value);
    [[nodiscard]] std::optional<ObservationStatus> observation_status() const;
    [[nodiscard]] std::optional<EffectorStatus> effector_status() const;
    [[nodiscard]] std::optional<CommandAck> acknowledgement(
        ComponentId source, std::uint32_t command_id) const;
    [[nodiscard]] std::vector<ErrorReport> errors() const;
private:
    static std::uint64_t ack_key(ComponentId source, std::uint32_t command_id) noexcept;
    std::size_t maximum_errors_;
    mutable std::mutex mutex_;
    std::optional<ObservationStatus> observation_;
    std::optional<EffectorStatus> effector_;
    std::unordered_map<std::uint64_t, CommandAck> acknowledgements_;
    std::deque<ErrorReport> errors_;
};
}  // namespace c2
