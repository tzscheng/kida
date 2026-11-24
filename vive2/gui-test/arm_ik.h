#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace avatar {
constexpr double pi = 3.14159265358979323846;

struct Vec3 {
    double x = 0, y = 0, z = 0;
    Vec3 operator+(Vec3 b) const { return {x+b.x, y+b.y, z+b.z}; }
    Vec3 operator-(Vec3 b) const { return {x-b.x, y-b.y, z-b.z}; }
    Vec3 operator*(double s) const { return {x*s, y*s, z*s}; }
    Vec3 operator/(double s) const { return *this * (1/s); }
};
inline double dot(Vec3 a, Vec3 b) { return a.x*b.x+a.y*b.y+a.z*b.z; }
inline Vec3 cross(Vec3 a, Vec3 b) {
    return {a.y*b.z-a.z*b.y, a.z*b.x-a.x*b.z, a.x*b.y-a.y*b.x};
}
inline double length(Vec3 v) { return std::sqrt(dot(v,v)); }
inline bool finite(Vec3 v) { return std::isfinite(v.x)&&std::isfinite(v.y)&&std::isfinite(v.z); }
inline Vec3 unit(Vec3 v, Vec3 fallback = {1,0,0}) {
    double n=length(v); return n>1e-10 ? v/n : fallback;
}
inline Vec3 lerp(Vec3 a, Vec3 b, double t) { return a+(b-a)*t; }

struct Quat {
    double x=0, y=0, z=0, w=1;
    Quat normalized() const {
        double n=std::sqrt(x*x+y*y+z*z+w*w);
        if (!(n>1e-10) || !std::isfinite(n)) return {};
        return {x/n,y/n,z/n,w/n};
    }
    Quat inverse() const { auto q=normalized(); return {-q.x,-q.y,-q.z,q.w}; }
    Quat operator*(Quat b) const {
        return {w*b.x+x*b.w+y*b.z-z*b.y, w*b.y-x*b.z+y*b.w+z*b.x,
                w*b.z+x*b.y-y*b.x+z*b.w, w*b.w-x*b.x-y*b.y-z*b.z};
    }
    Vec3 rotate(Vec3 v) const {
        auto q=normalized(); Vec3 u{q.x,q.y,q.z};
        return v+cross(u,v)*(2*q.w)+cross(u,cross(u,v))*2;
    }
    static Quat yaw(double angle) { return {0,0,std::sin(angle/2),std::cos(angle/2)}; }
};
inline Quat blend(Quat a, Quat b, double t) {
    a=a.normalized(); b=b.normalized();
    if (a.x*b.x+a.y*b.y+a.z*b.z+a.w*b.w<0) b={-b.x,-b.y,-b.z,-b.w};
    return Quat{a.x+(b.x-a.x)*t,a.y+(b.y-a.y)*t,a.z+(b.z-a.z)*t,a.w+(b.w-a.w)*t}.normalized();
}
// Extrinsic XYZ roll/pitch/yaw, R = Rz(yaw) Ry(pitch) Rx(roll), in degrees.
inline Vec3 rpy_degrees(Quat q) {
    q=q.normalized();
    double roll=std::atan2(2*(q.w*q.x+q.y*q.z),1-2*(q.x*q.x+q.y*q.y));
    double pitch=std::asin(std::clamp(2*(q.w*q.y-q.z*q.x),-1.,1.));
    double yaw=std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
    return Vec3{roll,pitch,yaw}*(180/pi);
}
struct Pose { Vec3 position; Quat rotation; };

struct Body {
    double height=1.75;
    double shoulder_z() const { return height*.83; }
};
} // namespace avatar
