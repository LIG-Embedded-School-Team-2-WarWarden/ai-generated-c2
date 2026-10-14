#pragma once
#include "c2/datagram_sender.hpp"
#include <condition_variable>
#include <chrono>
#include <deque>
#include <thread>

namespace c2 {
struct DatagramProcessorConfig {
    std::size_t maximum_queued_datagrams{1024};
    std::size_t maximum_queued_bytes{4 * 1024 * 1024};
};
struct InboundDatagram {
    std::vector<std::byte> bytes;
    Endpoint source;
    std::uint64_t received_at_us{};
};
enum class EnqueueResult { accepted, full, stopped };
struct DatagramProcessorStats {
    std::uint64_t accepted{};
    std::uint64_t processed{};
    std::uint64_t dropped_full{};
    std::uint64_t dropped_stopped{};
    std::uint64_t handler_errors{};
    std::uint64_t total_queue_wait_us{};
    std::uint64_t maximum_queue_wait_us{};
    std::size_t queued_datagrams{};
    std::size_t queued_bytes{};
    std::size_t high_water_datagrams{};
    std::size_t high_water_bytes{};
};
// FIFO single consumer. stop() closes admission, drains admitted work and joins.
// Lifecycle calls belong to the owner, never to the handler itself.
class DatagramProcessor final {
public:
    using Handler = std::function<void(InboundDatagram)>;
    DatagramProcessor(DatagramProcessorConfig config, Handler handler);
    ~DatagramProcessor();
    void start();
    void stop() noexcept;
    [[nodiscard]] EnqueueResult submit(InboundDatagram datagram);
    [[nodiscard]] DatagramProcessorStats stats() const;
private:
    void process() noexcept;
    DatagramProcessorConfig config_;
    Handler handler_;
    std::mutex lifecycle_mutex_;
    mutable std::mutex mutex_;
    std::condition_variable ready_;
    struct Entry {
        InboundDatagram datagram;
        std::chrono::steady_clock::time_point enqueued_at;
    };
    std::deque<Entry> queue_;
    DatagramProcessorStats stats_;
    bool accepting_{};
    std::thread worker_;
};
} // namespace c2
