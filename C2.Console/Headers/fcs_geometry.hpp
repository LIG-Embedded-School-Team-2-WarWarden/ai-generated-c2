#pragma once
#include "fcs_angles.hpp"
#include <array>
struct FcsGeometry {
    bool enabled{};
    std::array<double,3> translation{}, rotation{}; // metres, Rodrigues vector (radians).
    std::array<double,3> rdr_anchor{}, fcs_anchor{};
    bool operator==(const FcsGeometry&) const = default;
};
inline std::array<double,3> fcs_geometry_point(std::array<double,3> p,const FcsGeometry& g) {
    if(!g.enabled)return p;
    const auto& r=g.rotation;
    const double theta=std::hypot(r[0],r[1],r[2]);
    const double a=theta<1e-8?1-theta*theta/6:std::sin(theta)/theta;
    const double b=theta<1e-8?.5-theta*theta/24:(1-std::cos(theta))/(theta*theta);
    const std::array<double,3> cross{r[1]*p[2]-r[2]*p[1],r[2]*p[0]-r[0]*p[2],r[0]*p[1]-r[1]*p[0]};
    const std::array<double,3> twice{r[1]*cross[2]-r[2]*cross[1],r[2]*cross[0]-r[0]*cross[2],r[0]*cross[1]-r[1]*cross[0]};
    for(size_t i=0;i<3;++i)p[i]+=a*cross[i]+b*twice[i]+g.translation[i];
    return p;
}
inline FcsPointAngles fcs_calibrated_angles(double east,double north,double up,
    double pan_offset,double tilt_offset,const FcsGeometry& geometry) {
    const auto p=fcs_geometry_point({east,north,up},geometry);
    return fcs_point_angles(p[0],p[1],p[2],pan_offset,tilt_offset);
}
