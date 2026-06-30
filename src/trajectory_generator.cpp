#include "uav_pid_mavros/trajectory_generator.hpp"

#include <algorithm>
#include <cmath>

namespace uav_pid_mavros
{

namespace
{
constexpr double kTwoPi = 6.28318530717958647692;
}

Vec3 TrajectoryGenerator::hoverPoint(
  const Vec3 & home_position,
  double relative_height) const
{
  return Vec3(home_position.x, home_position.y, home_position.z + relative_height);
}

Vec3 TrajectoryGenerator::circlePoint(
  const Vec3 & circle_center,
  double radius,
  double theta_rad) const
{
  return Vec3(
    circle_center.x + radius * std::cos(theta_rad),
    circle_center.y + radius * std::sin(theta_rad),
    circle_center.z);
}

double TrajectoryGenerator::circleDuration(
  double radius,
  double linear_speed) const
{
  const double safe_speed = std::max(0.05, std::abs(linear_speed));
  return kTwoPi * radius / safe_speed;
}

}  // namespace uav_pid_mavros