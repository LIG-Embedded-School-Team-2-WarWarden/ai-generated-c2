#pragma once
#include "fcs_angles.hpp"
#include <array>
struct FcsGeometry {
    bool enabled{};
    // RDR-local point -> camera motor frame. Translation is metres and rotation
    // is a Rodrigues vector in radians. Absolute map installation is not part
    // of this transform.
    std::array<double,3> translation{}, rotation{};
    std::array<double,3> rdr_anchor{}, fcs_anchor{};
    bool operator==(const FcsGeometry&) const = default;
};
inline std::array<double,3> fcs_geometry_rotate(std::array<double,3> p,const FcsGeometry& g) {
    const auto& r=g.rotation;
    const double theta=std::hypot(r[0],r[1],r[2]);
    const double a=theta<1e-8?1-theta*theta/6:std::sin(theta)/theta;
    const double b=theta<1e-8?.5-theta*theta/24:(1-std::cos(theta))/(theta*theta);
    const std::array<double,3> cross{r[1]*p[2]-r[2]*p[1],r[2]*p[0]-r[0]*p[2],r[0]*p[1]-r[1]*p[0]};
    const std::array<double,3> twice{r[1]*cross[2]-r[2]*cross[1],r[2]*cross[0]-r[0]*cross[2],r[0]*cross[1]-r[1]*cross[0]};
    for(size_t i=0;i<3;++i)p[i]+=a*cross[i]+b*twice[i];
    return p;
}
inline std::array<double,3> fcs_geometry_point(std::array<double,3> p,const FcsGeometry& g) {
    if(!g.enabled)return p;
    p=fcs_geometry_rotate(p,g);
    for(size_t i=0;i<3;++i)p[i]+=g.translation[i];
    return p;
}
inline FcsPointAngles fcs_calibrated_angles(double east,double north,double up,
    double pan_offset,double tilt_offset,const FcsGeometry& geometry) {
    const auto p=fcs_geometry_point({east,north,up},geometry);
    return fcs_point_angles(p[0],p[1],p[2],pan_offset,tilt_offset);
}
inline std::array<double,3> fcs_gui_origin(const std::array<double,3>& installation_position) {
    // Installation storage is (forward,left,up); GUI/world is (east,north,up).
    return {-installation_position[1],installation_position[0],installation_position[2]};
}
inline std::array<double,3> fcs_target_vector(double world_east,double world_north,double world_up,
    const std::array<double,3>& rdr_position,const std::array<double,3>& fcs_position,
    const FcsGeometry& geometry) {
    const auto origin=fcs_gui_origin(geometry.enabled?rdr_position:fcs_position);
    std::array<double,3> point{world_east-origin[0],world_north-origin[1],world_up-origin[2]};
    return geometry.enabled?fcs_geometry_point(point,geometry):point;
}
inline FcsPointAngles fcs_target_angles(double world_east,double world_north,double world_up,
    const std::array<double,3>& rdr_position,const std::array<double,3>& fcs_position,
    double pan_offset,double tilt_offset,const FcsGeometry& geometry) {
    const auto point=fcs_target_vector(world_east,world_north,world_up,rdr_position,fcs_position,geometry);
    return fcs_point_angles(point[0],point[1],point[2],pan_offset,tilt_offset);
}
