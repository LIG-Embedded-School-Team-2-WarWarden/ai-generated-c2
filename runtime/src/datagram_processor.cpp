#include "c2/datagram_processor.hpp"
#include <algorithm>
#include <stdexcept>
#include <utility>
namespace c2 {
DatagramProcessor::DatagramProcessor(DatagramProcessorConfig config, Handler handler)
    : config_(config), handler_(std::move(handler)) {
    if (!handler_ || !config.maximum_queued_datagrams || !config.maximum_queued_bytes)
        throw std::invalid_argument("datagram processor requires a handler and positive limits");
}
DatagramProcessor::~DatagramProcessor() { stop(); }
void DatagramProcessor::start() {
    std::lock_guard lifecycle(lifecycle_mutex_);
    std::lock_guard lock(mutex_);
    if (worker_.joinable()) throw std::logic_error("datagram processor already started");
    accepting_ = true;
    try { worker_ = std::thread(&DatagramProcessor::process, this); }
    catch (...) { accepting_ = false; throw; }
}
void DatagramProcessor::stop() noexcept {
    std::lock_guard lifecycle(lifecycle_mutex_);
    { std::lock_guard lock(mutex_); accepting_ = false; }
    ready_.notify_all();
    if (worker_.joinable()) worker_.join();
}
EnqueueResult DatagramProcessor::submit(InboundDatagram datagram) {
    std::lock_guard lock(mutex_);
    if (!accepting_) { ++stats_.dropped_stopped; return EnqueueResult::stopped; }
    if (queue_.size() >= config_.maximum_queued_datagrams ||
        datagram.bytes.size() > config_.maximum_queued_bytes - stats_.queued_bytes) {
        ++stats_.dropped_full;
        return EnqueueResult::full;
    }
    const auto size = datagram.bytes.size();
    queue_.push_back({std::move(datagram), std::chrono::steady_clock::now()});
    ++stats_.accepted;
    stats_.queued_datagrams = queue_.size();
    stats_.queued_bytes += size;
    stats_.high_water_datagrams = std::max(stats_.high_water_datagrams, queue_.size());
    stats_.high_water_bytes = std::max(stats_.high_water_bytes, stats_.queued_bytes);
    ready_.notify_one();
    return EnqueueResult::accepted;
}
DatagramProcessorStats DatagramProcessor::stats() const {
    std::lock_guard lock(mutex_);
    return stats_;
}
void DatagramProcessor::process() noexcept {
    for (;;) {
        InboundDatagram datagram;
        {
            std::unique_lock lock(mutex_);
            ready_.wait(lock, [&] { return !queue_.empty() || !accepting_; });
            if (queue_.empty()) return;
            const auto wait_us = static_cast<std::uint64_t>(
                std::chrono::duration_cast<std::chrono::microseconds>(
                    std::chrono::steady_clock::now() - queue_.front().enqueued_at).count());
            stats_.total_queue_wait_us += wait_us;
            stats_.maximum_queue_wait_us = std::max(stats_.maximum_queue_wait_us, wait_us);
            datagram = std::move(queue_.front().datagram);
            queue_.pop_front();
            stats_.queued_datagrams = queue_.size();
            stats_.queued_bytes -= datagram.bytes.size();
        }
        bool failed{};
        try { handler_(std::move(datagram)); } catch (...) { failed = true; }
        {
            std::lock_guard lock(mutex_);
            ++stats_.processed;
            if (failed) ++stats_.handler_errors;
        }
    }
}
} // namespace c2
