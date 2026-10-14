#include "c2/event_log.hpp"
#include <chrono>
#include <filesystem>
#include <iomanip>
#include <random>
#include <sstream>
#include <stdexcept>

namespace c2 {
namespace {
std::string quote(const std::string& value) {
    std::ostringstream output;
    output << '"';
    for (const unsigned char ch : value) {
        if (ch == '"' || ch == '\\') output << '\\' << static_cast<char>(ch);
        else if (ch < 0x20) output << "\\u00" << std::hex << std::setw(2) << std::setfill('0') << unsigned(ch);
        else output << static_cast<char>(ch);
    }
    output << '"';
    return output.str();
}
}
void emit_event(const EventSink& sink, const RuntimeEvent& event) noexcept {
    try { if (sink) sink(event); } catch (...) {}
}
EventLog::EventLog(EventLogConfig config) : config_(std::move(config)) {
    if (config_.maximum_file_bytes < 1024 || config_.retained_files == 0 || config_.retained_files > 100)
        throw std::invalid_argument("event log requires >=1024 bytes and 1..100 retained files");
    run_id_ = std::to_string(std::chrono::system_clock::now().time_since_epoch().count()) +
              "-" + std::to_string(std::random_device{}());
    if (config_.path.empty()) return;
    const std::filesystem::path path(config_.path);
    if (!path.parent_path().empty()) std::filesystem::create_directories(path.parent_path());
    if (std::filesystem::exists(path)) bytes_ = std::filesystem::file_size(path);
    stream_.open(path, std::ios::binary | std::ios::app);
    if (!stream_) throw std::runtime_error("cannot open event log: " + config_.path);
}
void EventLog::rotate() {
    stream_.close();
    // Exact numbered files only: never remove directories or unrelated files.
    for (std::uint32_t i = config_.retained_files; i > 0; --i) {
        const auto destination = config_.path + "." + std::to_string(i);
        if (i == config_.retained_files && std::filesystem::exists(destination))
            std::filesystem::remove(destination);
        const auto source = i == 1 ? config_.path : config_.path + "." + std::to_string(i - 1);
        if (std::filesystem::exists(source)) std::filesystem::rename(source, destination);
    }
    stream_.clear();
    stream_.open(config_.path, std::ios::binary | std::ios::trunc);
    if (!stream_) throw std::runtime_error("cannot open rotated event log");
    bytes_ = 0;
    ++stats_.rotations;
}
void EventLog::append(const RuntimeEvent& event) noexcept {
    if (config_.path.empty()) return;
    std::lock_guard lock(mutex_);
    try {
        std::ostringstream line;
        line << "{\"schema_version\":1,\"run_id\":" << quote(run_id_)
             << ",\"event_id\":" << ++sequence_ << ",\"type\":" << quote(event.type)
             << ",\"recorded_at_us\":" << std::chrono::duration_cast<std::chrono::microseconds>(
                    std::chrono::system_clock::now().time_since_epoch()).count()
             << ",\"timestamp_us\":" << event.timestamp_us
             << ",\"asset_id\":" << event.asset_id << ",\"session_id\":" << event.session_id
             << ",\"command_id\":" << event.command_id << ",\"component_id\":" << event.component_id
             << ",\"detail\":" << quote(event.detail) << "}\n";
        const auto text = line.str();
        if (text.size() > config_.maximum_file_bytes) { ++stats_.write_errors; return; }
        if (bytes_ > config_.maximum_file_bytes - text.size()) rotate();
        stream_ << text;
        stream_.flush();
        if (!stream_) throw std::runtime_error("event log write failed");
        bytes_ += text.size();
        ++stats_.written;
    } catch (...) { ++stats_.write_errors; }
}
EventLogStats EventLog::stats() const {
    std::lock_guard lock(mutex_);
    return stats_;
}
} // namespace c2
