#pragma once

#include "c2/protocol.hpp"

#include <cstddef>
#include <cstdint>
#include <map>
#include <mutex>
#include <optional>
#include <tuple>
#include <vector>

namespace c2 {
struct TrackStoreConfig {
    std::uint64_t validity_us{2'000'000};
    std::size_t maximum_tracks{256};
    std::size_t maximum_tracks_per_observer{64};
    std::uint64_t first_track_id{1};
};

enum class TrackUpdateResult {
    stored,
    duplicate,
    stale,
    expired,
    invalid,
    capacity_exceeded,
    observer_capacity_exceeded,
};

struct TrackUpdate {
    TrackUpdateResult result{TrackUpdateResult::invalid};
    std::uint64_t track_id{};
};

struct TrackSnapshot {
    std::uint64_t track_id{};
    std::uint64_t observation_asset_id{};
    std::uint64_t observation_session_id{};
    std::uint32_t detection_id{};
    TargetCoordinate measurement;
    std::uint64_t received_at_us{};
    std::uint64_t expires_at_us{};
};

class TrackStore final {
public:
    explicit TrackStore(TrackStoreConfig config);

    [[nodiscard]] TrackUpdate update(
        const TargetCoordinate& measurement, std::uint64_t received_at_us);
    [[nodiscard]] std::optional<TrackSnapshot> track(
        std::uint64_t track_id, std::uint64_t now_us);
    [[nodiscard]] std::vector<TrackSnapshot> tracks(std::uint64_t now_us);
    std::size_t prune(std::uint64_t now_us);

private:
    using SourceKey = std::tuple<std::uint64_t, std::uint64_t, std::uint32_t>;

    [[nodiscard]] std::uint64_t allocate_track_id_locked();
    std::size_t prune_locked(std::uint64_t now_us);

    TrackStoreConfig config_;
    std::mutex mutex_;
    std::map<std::uint64_t, TrackSnapshot> tracks_;
    std::map<SourceKey, std::uint64_t> source_index_;
    std::uint64_t next_track_id_{};
};
}  // namespace c2
