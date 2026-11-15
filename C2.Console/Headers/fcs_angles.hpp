#pragma once
#include <cmath>
#include <numbers>
#include <stdexcept>
struct FcsPointAngles { double elevation{}, azimuth{}; };
inline double fcs_pan_error(double actual, double requested) {
    return std::abs(std::remainder(actual-requested,360.));
}
inline FcsPointAngles fcs_point_angles(double east, double north, double up,
                                      double pan_offset=0, double tilt_offset=0) {
    const double horizontal=std::hypot(east,north);
    if (!std::isfinite(horizontal) || !std::isfinite(up) || horizontal<.001 ||
        !std::isfinite(pan_offset) || !std::isfinite(tilt_offset))
        throw std::invalid_argument("undefined target bearing");
    // Device feedback: north=180 degrees, PAN increases toward the right.
    // Add optical alignment offsets once, before quantizing to the wire unit.
    double pan=std::round((180+std::atan2(east,north)*180/std::numbers::pi+pan_offset)*10)/10;
    pan=std::fmod(pan,360.);
    if(pan<0)pan+=360.;
    return {std::round((std::atan2(up,horizontal)*180/std::numbers::pi+tilt_offset)*10)/10,pan};
}
inline float fcs_motor_to_gui_azimuth(double motor) {
    auto degrees = std::fmod(std::round((motor-45)*10)/10, 360.);
    return float(degrees < 0 ? degrees+360 : degrees);
}
