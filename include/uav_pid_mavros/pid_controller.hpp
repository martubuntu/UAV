#pragma once

#include "uav_pid_mavros/types.hpp"

namespace uav_pid_mavros
{

struct AxisPidConfig
{
  double kp{0.0};
  double ki{0.0};
  double kd{0.0};
  double integral_limit{0.0};
  double output_limit{0.0};
};

class AxisPid
{
public:
  AxisPid() = default;

  void configure(const AxisPidConfig & config);
  void reset();
  double update(double error, double dt);

private:
  AxisPidConfig config_;
  double integral_{0.0};
  double previous_error_{0.0};
  bool first_update_{true};
};

class PidController
{
public:
  PidController() = default;

  void configure(
    const AxisPidConfig & x_config,
    const AxisPidConfig & y_config,
    const AxisPidConfig & z_config);

  void reset();

  Vec3 computeVelocityCommand(
    const Vec3 & target_position,
    const Vec3 & current_position,
    double dt);

private:
  AxisPid x_pid_;
  AxisPid y_pid_;
  AxisPid z_pid_;
};

}  // namespace uav_pid_mavros