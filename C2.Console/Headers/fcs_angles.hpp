#pragma once
#include <cmath>
#include <initializer_list>
#include <numbers>
#include <stdexcept>
struct FcsCalibration {
    double pan_offset_deg{}, tilt_offset_deg{};
    double yaw_deg{}, pitch_deg{}, roll_deg{};
    bool operator==(const FcsCalibration&) const = default;
};
inline void validate_fcs_calibration(const FcsCalibration& c) {
    for (double value : {c.pan_offset_deg,c.tilt_offset_deg,c.yaw_deg,c.pitch_deg,c.roll_deg})
        if (!std::isfinite(value) || std::abs(value)>180)
            throw std::invalid_argument("FCS calibration requires finite degrees within [-180,180]");
}
struct FcsPointAngles { double elevation{}, azimuth{}; };
inline FcsPointAngles fcs_point_angles(double east, double north, double up,
                                     const FcsCalibration& calibration = {}) {
    validate_fcs_calibration(calibration);
    // Inverse installation rotation: world -> mount. Positive yaw turns north
    // toward east; positive pitch raises the forward axis; positive roll raises
    // the right axis. Forward rotation order is roll, pitch, then yaw.
    const auto radians=[](double d) {return d*std::numbers::pi/180;};
    const double y=radians(calibration.yaw_deg), p=radians(calibration.pitch_deg),
                 r=radians(calibration.roll_deg);
    const double right=std::cos(y)*east-std::sin(y)*north;
    const double forward=std::sin(y)*east+std::cos(y)*north;
    const double level_forward=std::cos(p)*forward+std::sin(p)*up;
    const double level_up=-std::sin(p)*forward+std::cos(p)*up;
    east=std::cos(r)*right+std::sin(r)*level_up;
    north=level_forward;
    up=-std::sin(r)*right+std::cos(r)*level_up;
    const double horizontal=std::hypot(east,north);
    if (!std::isfinite(horizontal) || !std::isfinite(up) || horizontal<.001)
        throw std::invalid_argument("undefined target bearing");
    // Device feedback: north=180 degrees, PAN increases toward the right.
    // Quantize only once, after all corrections. Keep the existing motor angle
    // convention: do not assume hardware supports wrap-around or shortest paths.
    return {std::round((std::atan2(up,horizontal)*180/std::numbers::pi+calibration.tilt_offset_deg)*10)/10,
            std::round((180+std::atan2(east,north)*180/std::numbers::pi+calibration.pan_offset_deg)*10)/10};
}
inline float fcs_motor_to_gui_azimuth(double motor) {
    auto degrees = std::fmod(std::round((motor-45)*10)/10, 360.);
    return float(degrees < 0 ? degrees+360 : degrees);
}
