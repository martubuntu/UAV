#include "uav_pid_mavros/pid_controller.hpp"

#include <cmath>

namespace uav_pid_mavros
{

void AxisPid::configure(const AxisPidConfig & config)
{
  config_ = config;
  reset();
}

void AxisPid::reset()
{
  integral_ = 0.0;
  previous_error_ = 0.0;
  first_update_ = true;
}

double AxisPid::update(double error, double dt)
{
  if (dt <= 1e-6 || !std::isfinite(dt)) {
    return 0.0;
  }

  integral_ += error * dt;

  if (config_.integral_limit > 0.0) {
    integral_ = clampValue(integral_, -config_.integral_limit, config_.integral_limit);
  }

  double derivative = 0.0;
  if (!first_update_) {
    derivative = (error - previous_error_) / dt;
  }

  previous_error_ = error;
  first_update_ = false;

  double output =
    config_.kp * error +
    config_.ki * integral_ +
    config_.kd * derivative;

  if (config_.output_limit > 0.0) {
    output = clampValue(output, -config_.output_limit, config_.output_limit);
  }

  return output;
}

void PidController::configure(
  const AxisPidConfig & x_config,
  const AxisPidConfig & y_config,
  const AxisPidConfig & z_config)
{
  x_pid_.configure(x_config);
  y_pid_.configure(y_config);
  z_pid_.configure(z_config);
}

void PidController::reset()
{
  x_pid_.reset();
  y_pid_.reset();
  z_pid_.reset();
}

Vec3 PidController::computeVelocityCommand(
  const Vec3 & target_position,
  const Vec3 & current_position,
  double dt)
{
  const Vec3 error = target_position - current_position;

  return Vec3(
    x_pid_.update(error.x, dt),
    y_pid_.update(error.y, dt),
    z_pid_.update(error.z, dt));
}

}  // namespace uav_pid_mavros