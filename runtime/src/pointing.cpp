#include "c2/pointing.hpp"

#include "c2/protocol_validation.hpp"

#include <cmath>
#include <limits>
#include <numbers>

namespace c2 {
namespace {
float degrees(const float radians) noexcept {
    return radians * 180.0F / std::numbers::pi_v<float>;
}

float radians(const float degrees_value) noexcept {
    return degrees_value * std::numbers::pi_v<float> / 180.0F;
}

float normalize_pan(float angle) noexcept {
    while (angle >= 180.0F) angle -= 360.0F;
    while (angle < -180.0F) angle += 360.0F;
    return angle;
}
}  // namespace

Velocity3 rotate_velocity_to_project(
    const Velocity3 local, const float sensor_azimuth_deg) {
    if (!std::isfinite(local.x_mps) || !std::isfinite(local.y_mps) ||
        !std::isfinite(local.z_mps) || !std::isfinite(sensor_azimuth_deg))
        return {std::numeric_limits<float>::quiet_NaN(),
                std::numeric_limits<float>::quiet_NaN(),
                std::numeric_limits<float>::quiet_NaN()};
    const auto angle = radians(sensor_azimuth_deg);
    return {std::cos(angle) * local.x_mps - std::sin(angle) * local.y_mps,
            std::sin(angle) * local.x_mps + std::cos(angle) * local.y_mps,
            local.z_mps};
}

std::variant<PointingSolution, PointingError> calculate_effector_pointing(
    const TargetCoordinate& target,
    const AssetPose& effector,
    const PointingLimits& limits) {
    if (!validate(target).valid()) return PointingError::invalid_target;
    if (!validate(effector).valid() || effector.header.source_id != ComponentId::effector_asset)
        return PointingError::invalid_effector_pose;

    const auto world_x = target.x_m - effector.x_m;
    const auto world_y = target.y_m - effector.y_m;
    const auto world_z = target.z_m - effector.z_m;

    // PROJECT_FRAME -> effector body frame: rotate by the negative installation azimuth.
    const auto azimuth = radians(effector.azimuth_deg);
    PointingSolution result;
    result.relative_x_m = std::cos(azimuth) * world_x + std::sin(azimuth) * world_y;
    result.relative_y_m = -std::sin(azimuth) * world_x + std::cos(azimuth) * world_y;
    result.relative_z_m = world_z;

    const auto horizontal_distance = std::hypot(result.relative_x_m, result.relative_y_m);
    if (horizontal_distance < 1.0e-6F && std::abs(result.relative_z_m) < 1.0e-6F)
        return PointingError::coincident_target;

    result.pan_deg = normalize_pan(degrees(std::atan2(result.relative_y_m, result.relative_x_m)));
    result.tilt_deg = degrees(std::atan2(result.relative_z_m, horizontal_distance));
    if (result.pan_deg < limits.minimum_pan_deg || result.pan_deg > limits.maximum_pan_deg ||
        result.tilt_deg < limits.minimum_tilt_deg || result.tilt_deg > limits.maximum_tilt_deg)
        return PointingError::outside_turret_limits;
    return result;
}

}  // namespace c2
