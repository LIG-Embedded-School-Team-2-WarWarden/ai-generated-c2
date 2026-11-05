#pragma once
#include <cmath>
inline float fcs_motor_to_gui_azimuth(double motor) {
    auto degrees = std::fmod(std::round((315-motor)*10)/10, 360.);
    return float(degrees < 0 ? degrees+360 : degrees);
}
