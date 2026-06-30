#pragma once

#include <algorithm>
#include <cmath>

namespace uav_pid_mavros
{

struct Vec3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};

  Vec3() = default;
  Vec3(double x_in, double y_in, double z_in)
  : x(x_in), y(y_in), z(z_in)
  {}
};

inline Vec3 operator+(const Vec3 & a, const Vec3 & b)
{
  return Vec3(a.x + b.x, a.y + b.y, a.z + b.z);
}

inline Vec3 operator-(const Vec3 & a, const Vec3 & b)
{
  return Vec3(a.x - b.x, a.y - b.y, a.z - b.z);
}

inline Vec3 operator*(const Vec3 & a, double s)
{
  return Vec3(a.x * s, a.y * s, a.z * s);
}

inline double clampValue(double v, double lo, double hi)
{
  return std::max(lo, std::min(v, hi));
}

inline double norm3D(const Vec3 & v)
{
  return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

inline double distance3D(const Vec3 & a, const Vec3 & b)
{
  return norm3D(a - b);
}

}  // namespace uav_pid_mavros