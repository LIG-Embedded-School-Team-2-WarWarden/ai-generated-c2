#pragma once
#include "c2/rdr_session.hpp"
#include <cmath>
#include <limits>
#include <numbers>
#include <optional>

namespace c2::rdr {
// Installation baseline: aligned sensor/motor axes, coincident origins.
// North is wire PAN=1800. Positive PAN turns right; positive TILT turns up.
// Mounting rotation/lever arms require measured calibration before changing this baseline.
struct BasePoint { double east{}, north{}, up{}; };
struct CorrectedTarget {
    uint16_t id{};
    uint8_t status{2};
    BasePoint relative{}, world{}; // RDR-relative and WORLD (east,north,up), metres.
    bool valid{};
};
inline bool gui_point(const BasePoint& point) {
    const double limit = std::numeric_limits<float>::max();
    for (auto value : {point.east, point.north, point.up,
                       std::hypot(point.east, point.north, point.up)})
        if (!std::isfinite(value) || std::abs(value) > limit) return false;
    return true;
}
inline double heading_degrees(double motor_pan_degrees) {
    auto value = std::fmod(motor_pan_degrees, 360.);
    if (value < 0) value += 360.;
    return value - 180.;
}
inline std::optional<BasePoint> sensor_to_base(const Target& target) {
    if (target.status != 0 || target.track_state == 0 ||
        target.pan == -32768 || target.tilt == -32768) return {};
    const double pan = heading_degrees(target.pan / 10.) * std::numbers::pi / 180.;
    const double tilt = target.tilt / 10. * std::numbers::pi / 180.;
    // The lidar's reported X is mirrored: normalize to right-positive once,
    // before applying observation pose. Keep the received raw XYZ unchanged.
    const double right = -target.x;
    const double forward = target.z * std::cos(tilt) - target.y * std::sin(tilt);
    BasePoint result{right * std::cos(pan) + forward * std::sin(pan),
                     -right * std::sin(pan) + forward * std::cos(pan),
                     target.y * std::cos(tilt) + target.z * std::sin(tilt)};
    if (!gui_point(result)) return {};
    return result;
}
// Convert each observation using its own pose. Current motor status must never
// replace the pose carried by the observation.
inline std::vector<CorrectedTarget> correct_targets(const State& state, BasePoint origin) {
    std::vector<CorrectedTarget> result;
    for (const auto& [id, raw] : state.targets) {
        CorrectedTarget target;target.id=id;target.status=raw.status==3?3:2;
        if (const auto relative=sensor_to_base(raw)) {
            target.relative=*relative;
            target.world={relative->east+origin.east,relative->north+origin.north,relative->up+origin.up};
            target.valid=gui_point(target.world);
            if (target.valid) target.status=0;
        }
        result.push_back(target);
    }
    std::stable_sort(result.begin(),result.end(),[](const auto& a,const auto& b){return a.valid>b.valid;});
    return result;
}
} // namespace c2::rdr
