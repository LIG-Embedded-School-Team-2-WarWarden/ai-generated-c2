#pragma once
#include <cstdint>
#include <functional>
#include <fstream>
#include <mutex>
#include <string>

namespace c2 {
struct RuntimeEvent {
    std::string type;
    std::uint64_t timestamp_us{};
    std::uint64_t asset_id{};
    std::uint64_t session_id{};
    std::uint32_t command_id{};
    std::string detail;
    std::uint32_t component_id{};
};
using EventSink = std::function<void(const RuntimeEvent&)>;
// Observability must never change a command's state transition or throw to its caller.
void emit_event(const EventSink& sink, const RuntimeEvent& event) noexcept;
struct EventLogConfig {
    std::string path;
    std::uint64_t maximum_file_bytes{8 * 1024 * 1024};
    std::uint32_t retained_files{3};
};
struct EventLogStats {
    std::uint64_t written{};
    std::uint64_t write_errors{};
    std::uint64_t rotations{};
};
// One writer per file. Append + flush per event; bounded rotation, no fsync guarantee.
class EventLog final {
public:
    explicit EventLog(EventLogConfig config);
    void append(const RuntimeEvent& event) noexcept;
    [[nodiscard]] EventLogStats stats() const;
private:
    void rotate();
    EventLogConfig config_;
    mutable std::mutex mutex_;
    std::ofstream stream_;
    std::uint64_t bytes_{};
    std::uint64_t sequence_{};
    std::string run_id_;
    EventLogStats stats_;
};
} // namespace c2
