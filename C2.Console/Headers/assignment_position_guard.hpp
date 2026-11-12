#pragma once
#include "rdr_coordinates.hpp"
#include <array>
#include <algorithm>
#include <cmath>
#include <optional>

// Assignment continuity only: never changes filtered positions or sends commands.
// Keep the assignment-time reference so slow cumulative motion cannot creep past it.
class AssignmentPositionGuard {
    std::array<double,3> tolerance_;
    std::optional<c2::rdr::BasePoint> reference_;
    double last_sample_{};
    unsigned outside_samples_{};
    bool consistent_{};
public:
    const auto& reference() const { return reference_; }
    const auto& tolerance() const { return tolerance_; }
    unsigned outside_samples() const { return outside_samples_; }
    explicit AssignmentPositionGuard(std::array<double,3> measurement_stddev) {
        for (size_t axis=0;axis<3;++axis)
            tolerance_[axis]=std::max(.05,3.*measurement_stddev[axis]);
    }
    void reset() { reference_.reset();outside_samples_=0;consistent_=false; }
    void start(c2::rdr::BasePoint point,double sample_time) {
        reference_=point;last_sample_=sample_time;outside_samples_=0;consistent_=true;
    }
    // A generated position is not an additional observed sample. Compare its
    // displacement without modifying the observed-sample debounce history.
    bool prediction_consistent(c2::rdr::BasePoint point) const {
        return reference_ && c2::rdr::gui_point(point) &&
            std::abs(point.east-reference_->east)<=tolerance_[0] &&
            std::abs(point.north-reference_->north)<=tolerance_[1] &&
            std::abs(point.up-reference_->up)<=tolerance_[2];
    }
    bool consistent(c2::rdr::BasePoint point,double sample_time) {
        if (!reference_ || !c2::rdr::gui_point(point) || !std::isfinite(sample_time)) return false;
        // GUI frames and status requests must not turn one outlier into three samples.
        if (sample_time==last_sample_) return consistent_;
        last_sample_=sample_time;
        const std::array<double,3> delta{point.east-reference_->east,
            point.north-reference_->north,point.up-reference_->up};
        const bool outside=std::abs(delta[0])>tolerance_[0] ||
            std::abs(delta[1])>tolerance_[1] || std::abs(delta[2])>tolerance_[2];
        outside_samples_=outside?std::min(outside_samples_+1,3u):0;
        consistent_=outside_samples_<3;
        return consistent_;
    }
};
