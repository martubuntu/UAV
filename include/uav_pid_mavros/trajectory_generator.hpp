#pragma once

#include "uav_pid_mavros/types.hpp"

namespace uav_pid_mavros
{

class TrajectoryGenerator
{
public:
  TrajectoryGenerator() = default;

  Vec3 hoverPoint(const Vec3 & home_position, double relative_height) const;

  Vec3 circlePoint(
    const Vec3 & circle_center,
    double radius,
    double theta_rad) const;

  double circleDuration(double radius, double linear_speed) const;
};

}  // namespace uav_pid_mavros