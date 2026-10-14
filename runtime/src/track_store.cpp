#include "c2/track_store.hpp"

#include "c2/protocol_validation.hpp"

#include <limits>
#include <stdexcept>

namespace c2 {
namespace {
std::uint64_t deadline(
    const std::uint64_t start, const std::uint64_t duration) noexcept {
    if (duration > std::numeric_limits<std::uint64_t>::max() - start)
        return std::numeric_limits<std::uint64_t>::max();
    return start + duration;
}

std::uint64_t next_non_zero(const std::uint64_t value) noexcept {
    return value == std::numeric_limits<std::uint64_t>::max() ? 1 : value + 1;
}
}

TrackStore::TrackStore(const TrackStoreConfig config)
    : config_(config), next_track_id_(config.first_track_id) {
    if (config_.validity_us == 0)
        throw std::invalid_argument("track validity must be non-zero");
    if (config_.maximum_tracks == 0)
        throw std::invalid_argument("maximum track count must be non-zero");
    if (config_.maximum_tracks_per_observer == 0 ||
        config_.maximum_tracks_per_observer > config_.maximum_tracks)
        throw std::invalid_argument("per-observer track limit is invalid");
    if (config_.first_track_id == 0)
        throw std::invalid_argument("first track ID must be non-zero");
}

TrackUpdate TrackStore::update(
    const TargetCoordinate& measurement, const std::uint64_t received_at_us) {
    if (!validate(measurement).valid() || received_at_us == 0)
        return {TrackUpdateResult::invalid, 0};
    if (received_at_us >= measurement.measurement_time_us &&
        received_at_us - measurement.measurement_time_us >= config_.validity_us)
        return {TrackUpdateResult::expired, 0};

    std::lock_guard lock(mutex_);
    (void)prune_locked(received_at_us);
    const SourceKey key{measurement.header.asset_id,
                        measurement.header.session_id,
                        measurement.detection_id};
    const auto existing = source_index_.find(key);
    if (existing != source_index_.end()) {
        auto& track = tracks_.at(existing->second);
        if (measurement.measurement_time_us ==
                track.measurement.measurement_time_us &&
            measurement.header.sequence == track.measurement.header.sequence)
            return {TrackUpdateResult::duplicate, track.track_id};
        if (measurement.measurement_time_us <=
            track.measurement.measurement_time_us)
            return {TrackUpdateResult::stale, track.track_id};
        track.measurement = measurement;
        track.received_at_us = received_at_us;
        track.expires_at_us = deadline(received_at_us, config_.validity_us);
        return {TrackUpdateResult::stored, track.track_id};
    }

    if (tracks_.size() >= config_.maximum_tracks)
        return {TrackUpdateResult::capacity_exceeded, 0};
    std::size_t observer_tracks{};
    for (const auto& [source, id] : source_index_) {
        (void)id;
        if (std::get<0>(source) == measurement.header.asset_id)
            ++observer_tracks;
    }
    if (observer_tracks >= config_.maximum_tracks_per_observer)
        return {TrackUpdateResult::observer_capacity_exceeded, 0};

    const auto track_id = allocate_track_id_locked();
    TrackSnapshot snapshot{track_id,
                           measurement.header.asset_id,
                           measurement.header.session_id,
                           measurement.detection_id,
                           measurement,
                           received_at_us,
                           deadline(received_at_us, config_.validity_us)};
    tracks_.emplace(track_id, snapshot);
    source_index_.emplace(key, track_id);
    return {TrackUpdateResult::stored, track_id};
}

std::uint64_t TrackStore::allocate_track_id_locked() {
    auto candidate = next_track_id_;
    for (std::size_t attempt = 0; attempt <= tracks_.size(); ++attempt) {
        if (!tracks_.contains(candidate)) {
            next_track_id_ = next_non_zero(candidate);
            return candidate;
        }
        candidate = next_non_zero(candidate);
    }
    throw std::overflow_error("no track identifier is available");
}

std::size_t TrackStore::prune_locked(const std::uint64_t now_us) {
    std::size_t removed{};
    for (auto iterator = tracks_.begin(); iterator != tracks_.end();) {
        if (now_us >= iterator->second.expires_at_us) {
            const auto& track = iterator->second;
            source_index_.erase(SourceKey{track.observation_asset_id,
                                          track.observation_session_id,
                                          track.detection_id});
            iterator = tracks_.erase(iterator);
            ++removed;
        } else {
            ++iterator;
        }
    }
    return removed;
}

std::optional<TrackSnapshot> TrackStore::track(
    const std::uint64_t track_id, const std::uint64_t now_us) {
    std::lock_guard lock(mutex_);
    (void)prune_locked(now_us);
    const auto found = tracks_.find(track_id);
    if (found == tracks_.end()) return std::nullopt;
    return found->second;
}

std::vector<TrackSnapshot> TrackStore::tracks(const std::uint64_t now_us) {
    std::lock_guard lock(mutex_);
    (void)prune_locked(now_us);
    std::vector<TrackSnapshot> result;
    result.reserve(tracks_.size());
    for (const auto& [id, track] : tracks_) {
        (void)id;
        result.push_back(track);
    }
    return result;
}

std::size_t TrackStore::prune(const std::uint64_t now_us) {
    std::lock_guard lock(mutex_);
    return prune_locked(now_us);
}
}  // namespace c2
