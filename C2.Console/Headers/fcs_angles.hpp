#pragma once
#include <cmath>
#include <numbers>
#include <stdexcept>
struct FcsPointAngles { double elevation{}, azimuth{}; };
inline FcsPointAngles fcs_point_angles(double east, double north, double up) {
    const double horizontal=std::hypot(east,north);
    if (!std::isfinite(horizontal) || !std::isfinite(up) || horizontal<.001)
        throw std::invalid_argument("undefined target bearing");
    // Device feedback: north=180 degrees, PAN increases toward the right.
    return {std::round(std::atan2(up,horizontal)*180/std::numbers::pi*10)/10,
            std::round((180+std::atan2(east,north)*180/std::numbers::pi)*10)/10};
}
inline float fcs_motor_to_gui_azimuth(double motor) {
    auto degrees = std::fmod(std::round((motor-45)*10)/10, 360.);
    return float(degrees < 0 ? degrees+360 : degrees);
}
