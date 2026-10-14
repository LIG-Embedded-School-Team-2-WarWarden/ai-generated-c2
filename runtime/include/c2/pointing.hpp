#pragma once

#include "c2/protocol.hpp"

#include <variant>

namespace c2 {

struct PointingLimits {
    float minimum_pan_deg{-180.0F};
    float maximum_pan_deg{180.0F};
    float minimum_tilt_deg{-90.0F};
    float maximum_tilt_deg{90.0F};
};

struct PointingSolution {
    float relative_x_m{};
    float relative_y_m{};
    float relative_z_m{};
    float pan_deg{};
    float tilt_deg{};
};

struct Velocity3 {
    float x_mps{};
    float y_mps{};
    float z_mps{};
};

// Rotates a sensor-local velocity into PROJECT_FRAME. Velocity is a vector,
// so installation translation is deliberately not applied.
[[nodiscard]] Velocity3 rotate_velocity_to_project(
    Velocity3 local_velocity, float sensor_azimuth_deg);

enum class PointingError {
    invalid_target,
    invalid_effector_pose,
    coincident_target,
    outside_turret_limits,
};

[[nodiscard]] std::variant<PointingSolution, PointingError> calculate_effector_pointing(
    const TargetCoordinate& project_target,
    const AssetPose& effector_pose,
    const PointingLimits& limits = {});

}  // namespace c2
