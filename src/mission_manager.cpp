
#include "uav_pid_mavros/mission_manager.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace uav_pid_mavros
{

namespace
{
constexpr double kTwoPi = 6.28318530717958647692;
}

MissionManager::MissionManager(const rclcpp::Node::SharedPtr & node)
: node_(node)
{
  control_rate_hz_ = node_->declare_parameter<double>("control_rate_hz", 30.0);

  hover_time_ = node_->declare_parameter<double>("hover_time", 10.0);
  pre_task_hold_time_ = node_->declare_parameter<double>("pre_task_hold_time", 5.0);
  final_hold_time_ = node_->declare_parameter<double>("final_hold_time", 5.0);

  circle_radius_ = node_->declare_parameter<double>("circle_radius", 1.5);
  circle_speed_ = node_->declare_parameter<double>("circle_speed", 0.18);

  circle_error_slow_ = node_->declare_parameter<double>("circle_error_slow", 0.25);
  circle_error_very_slow_ = node_->declare_parameter<double>("circle_error_very_slow", 0.50);

  circle_slow_scale_ =
    clampValue(node_->declare_parameter<double>("circle_slow_scale", 0.60), 0.0, 1.0);

  circle_min_scale_ =
    clampValue(node_->declare_parameter<double>("circle_min_scale", 0.30), 0.0, 1.0);

  line_speed_ = node_->declare_parameter<double>("line_speed", 0.20);

  square_side_ = node_->declare_parameter<double>("square_side", 2.0);
  square_hold_time_ = node_->declare_parameter<double>("square_hold_time", 5.0);
  square_step_ = node_->declare_parameter<double>("square_step", 0.10);
  square_target_tolerance_ =
    node_->declare_parameter<double>("square_target_tolerance", 0.08);
  square_target_update_period_ =
    node_->declare_parameter<double>("square_target_update_period", 0.20);

  position_tolerance_ = node_->declare_parameter<double>("position_tolerance", 0.18);
  settle_time_ = node_->declare_parameter<double>("settle_time", 0.8);
  landed_height_threshold_ = node_->declare_parameter<double>("landed_height_threshold", 0.15);

  hover_max_vxy_ = node_->declare_parameter<double>("hover_max_vxy", 0.20);
  square_max_vxy_ = node_->declare_parameter<double>("square_max_vxy", 0.22);
  circle_max_vxy_ = node_->declare_parameter<double>("circle_max_vxy", 0.30);

  max_vxy_ = hover_max_vxy_;

  max_vz_up_ = node_->declare_parameter<double>("max_vz_up", 0.22);
  max_vz_down_ = node_->declare_parameter<double>("max_vz_down", 0.16);

  altitude_kp_ = node_->declare_parameter<double>("altitude_kp", 0.55);
  altitude_ki_ = node_->declare_parameter<double>("altitude_ki", 0.08);
  altitude_kd_ = node_->declare_parameter<double>("altitude_kd", 0.10);
  altitude_deadband_ = node_->declare_parameter<double>("altitude_deadband", 0.04);
  altitude_integral_limit_ =
    node_->declare_parameter<double>("altitude_integral_limit", 0.35);

  altitude_derivative_alpha_ =
    clampValue(
      node_->declare_parameter<double>("altitude_derivative_alpha", 0.25),
      0.0,
      1.0);

  auto_offboard_ = node_->declare_parameter<bool>("auto_offboard", false);
  auto_arm_ = node_->declare_parameter<bool>("auto_arm", false);

  mission_phase_ = node_->declare_parameter<std::string>("mission_phase", "full");
  finish_action_ = node_->declare_parameter<std::string>("finish_action", "land");

  service_request_period_ =
    node_->declare_parameter<double>("service_request_period", 1.0);

  // 【新增】偏移距离参数读取（直接改这里默认值即可生效）
  offset_distance_x_ = node_->declare_parameter<double>("offset_distance_x", 3.0);
  offset_distance_y_ = node_->declare_parameter<double>("offset_distance_y", 0.0);

    hover_x_config_ = AxisPidConfig{
    node_->declare_parameter<double>("hover_pid_x_kp", 0.55),
    node_->declare_parameter<double>("hover_pid_x_ki", 0.0),
    node_->declare_parameter<double>("hover_pid_x_kd", 0.10),
    node_->declare_parameter<double>("hover_pid_x_integral_limit", 0.3),
    hover_max_vxy_
  };

  hover_y_config_ = AxisPidConfig{
    node_->declare_parameter<double>("hover_pid_y_kp", 0.55),
    node_->declare_parameter<double>("hover_pid_y_ki", 0.0),
    node_->declare_parameter<double>("hover_pid_y_kd", 0.10),
    node_->declare_parameter<double>("hover_pid_y_integral_limit", 0.3),
    hover_max_vxy_
  };

  square_x_config_ = AxisPidConfig{
    node_->declare_parameter<double>("square_pid_x_kp", 0.60),
    node_->declare_parameter<double>("square_pid_x_ki", 0.0),
    node_->declare_parameter<double>("square_pid_x_kd", 0.12),
    node_->declare_parameter<double>("square_pid_x_integral_limit", 0.3),
    square_max_vxy_
  };

  square_y_config_ = AxisPidConfig{
    node_->declare_parameter<double>("square_pid_y_kp", 0.60),
    node_->declare_parameter<double>("square_pid_y_ki", 0.0),
    node_->declare_parameter<double>("square_pid_y_kd", 0.12),
    node_->declare_parameter<double>("square_pid_y_integral_limit", 0.3),
    square_max_vxy_
  };

  circle_x_config_ = AxisPidConfig{
    node_->declare_parameter<double>("circle_pid_x_kp", 0.70),
    node_->declare_parameter<double>("circle_pid_x_ki", 0.0),
    node_->declare_parameter<double>("circle_pid_x_kd", 0.10),
    node_->declare_parameter<double>("circle_pid_x_integral_limit", 0.3),
    circle_max_vxy_
  };

  circle_y_config_ = AxisPidConfig{
    node_->declare_parameter<double>("circle_pid_y_kp", 0.70),
    node_->declare_parameter<double>("circle_pid_y_ki", 0.0),
    node_->declare_parameter<double>("circle_pid_y_kd", 0.10),
    node_->declare_parameter<double>("circle_pid_y_integral_limit", 0.3),
    circle_max_vxy_
  };

  z_config_ = AxisPidConfig{
    node_->declare_parameter<double>("pid_z_kp", 0.25),
    node_->declare_parameter<double>("pid_z_ki", 0.0),
    node_->declare_parameter<double>("pid_z_kd", 0.05),
    node_->declare_parameter<double>("pid_z_integral_limit", 0.05),
    std::max(max_vz_up_, max_vz_down_)
  };

  configureControlProfile(MissionState::HOVER);

  mavros_ = std::make_unique<MavrosInterface>(node_);

  // 【修改】校验条件加入 "offset_land"
  if (
    mission_phase_ != "hover_only" &&
    mission_phase_ != "square_only" &&
    mission_phase_ != "circle_only" &&
    mission_phase_ != "full" &&
    mission_phase_ != "offset_land")
  {
    RCLCPP_WARN(
      node_->get_logger(),
      "Unknown mission_phase='%s'. Falling back to hover_only.",
      mission_phase_.c_str());

    mission_phase_ = "hover_only";
  }

  if (finish_action_ != "hold" && finish_action_ != "land") {
    RCLCPP_WARN(
      node_->get_logger(),
      "Unknown finish_action='%s'. Falling back to hold.",
      finish_action_.c_str());

    finish_action_ = "hold";
  }

  if (auto_offboard_ || auto_arm_) {
    RCLCPP_WARN(
      node_->get_logger(),
      "Manual ARM and manual OFFBOARD are expected. auto_offboard/auto_arm are ignored.");
  }
}

void MissionManager::start()
{
  const auto period_ms =
    std::chrono::milliseconds(static_cast<int>(1000.0 / std::max(1.0, control_rate_hz_)));

  last_update_sec_ = node_->now().seconds();
  state_enter_sec_ = last_update_sec_;

  timer_ = node_->create_wall_timer(
    period_ms,
    std::bind(&MissionManager::update, this));

  RCLCPP_INFO(node_->get_logger(), "UAV PID MAVROS mission manager started.");
  RCLCPP_INFO(node_->get_logger(), "Manual ARM and manual OFFBOARD mode are expected.");
}

void MissionManager::update()
{
  const double now_sec = node_->now().seconds();
  double dt = now_sec - last_update_sec_;
  last_update_sec_ = now_sec;

  if (dt <= 1e-6 || dt > 0.5 || !std::isfinite(dt)) {
    dt = 1.0 / std::max(1.0, control_rate_hz_);
  }

  Vec3 reference_position;
  Vec3 velocity_cmd;

  if (!mavros_->connected()) {
    return;
  }

  if (!mavros_->hasLocalPose()) {
    return;
  }

  const Vec3 current_position = mavros_->currentPosition();
  reference_position = current_position;

  if (
    state_ != MissionState::WAIT_FOR_FCU &&
    state_ != MissionState::WAIT_FOR_MANUAL_OFFBOARD &&
     state_ != MissionState::LAND)
  {
    if (!mavros_->armed() || mavros_->mode() != "OFFBOARD") {
      home_locked_ = false;
      pid_.reset();
      resetAltitudeController();

      RCLCPP_WARN(
        node_->get_logger(),
        "OFFBOARD or ARM lost. Stop mission control and wait for manual OFFBOARD again.");

      transitionTo(MissionState::WAIT_FOR_MANUAL_OFFBOARD, now_sec);
      mavros_->publishVelocity(Vec3(), activeYaw());
      return;
    }

  }

  switch (state_) {
    case MissionState::WAIT_FOR_FCU:
    {
      RCLCPP_INFO(
        node_->get_logger(),
        "FCU connected and local pose received. Waiting for manual ARM and manual OFFBOARD.");

      transitionTo(MissionState::WAIT_FOR_MANUAL_OFFBOARD, now_sec);
      break;
    }

    case MissionState::WAIT_FOR_MANUAL_OFFBOARD:
    {
      reference_position = current_position;
      velocity_cmd = Vec3();

      if (mavros_->armed() && mavros_->mode() == "OFFBOARD") {
        lockHomePoint(current_position);
        transitionTo(MissionState::HOVER, now_sec);
      }

      break;
    }

    case MissionState::HOVER:
    {
      reference_position = home_position_;

      velocity_cmd = computeMissionVelocityCommand(
        reference_position,
        current_position,
        dt);

      const double required_hold_time =
        (mission_phase_ == "hover_only") ? hover_time_ : pre_task_hold_time_;

      if ((now_sec - state_enter_sec_) >= required_hold_time) {
        if (mission_phase_ == "hover_only") {
          finishTask(now_sec);
        } else if (mission_phase_ == "square_only" || mission_phase_ == "full") {
          transitionTo(MissionState::SQUARE, now_sec);
        } else if (mission_phase_ == "circle_only") {
          transitionTo(MissionState::GO_TO_CIRCLE_START, now_sec);
        } else if (mission_phase_ == "offset_land") {
          // 【新增】悬停结束后进入偏移直飞
          transitionTo(MissionState::GO_TO_OFFSET, now_sec);
        }
      }

      break;
    }

    case MissionState::SQUARE:
    {
      if (square_targets_.empty()) {
        finishTask(now_sec);
        break;
      }

      reference_position = square_targets_[square_target_index_];

      velocity_cmd = computeMissionVelocityCommand(
        reference_position,
        current_position,
        dt);

      const double err = xyError(current_position, reference_position);
      const bool target_reached = err <= square_target_tolerance_;
      const bool update_period_ok =
        (now_sec - last_square_target_advance_sec_) >= square_target_update_period_;

      if (target_reached && update_period_ok) {
        const bool hold_here = square_hold_flags_[square_target_index_];

        if (hold_here) {
          if (square_point_arrive_sec_ < 0.0) {
            square_point_arrive_sec_ = now_sec;
            pid_.reset();
            resetAltitudeController();
          }

          if ((now_sec - square_point_arrive_sec_) < square_hold_time_) {
            break;
          }
        }

        square_target_index_++;
        square_point_arrive_sec_ = -1.0;
        last_square_target_advance_sec_ = now_sec;

        if (square_target_index_ >= square_targets_.size()) {
          hold_position_ = home_position_;

          if (mission_phase_ == "full") {
            transitionTo(MissionState::GO_TO_CIRCLE_START, now_sec);
          } else {
            finishTask(now_sec);
          }
        }
      }

      break;
    }

    case MissionState::GO_TO_CIRCLE_START:
    {
      if (!go_to_circle_start_initialized_) {
        go_to_circle_start_begin_ = current_position;
        go_to_circle_start_initialized_ = true;
      }

      const double elapsed = now_sec - state_enter_sec_;

      reference_position = lineReference(
        go_to_circle_start_begin_,
        circle_start_position_,
        elapsed,
        line_speed_);

      velocity_cmd = computeMissionVelocityCommand(
        reference_position,
        current_position,
        dt);

      const double duration = lineDuration(
        go_to_circle_start_begin_,
        circle_start_position_,
        line_speed_);

      if (
        elapsed >= duration &&
        stableAtXY(current_position, circle_start_position_, position_tolerance_, now_sec))
      {
        transitionTo(MissionState::CIRCLE, now_sec);
      }

      break;
    }

    case MissionState::CIRCLE:
    {
      const double omega =
        std::max(0.05, circle_speed_) / std::max(0.05, circle_radius_);

      reference_position = Vec3(
        circle_center_.x +
          circle_radius_ * (
            std::cos(circle_theta_) * circle_forward_.x +
            std::sin(circle_theta_) * circle_left_.x),
        circle_center_.y +
          circle_radius_ * (
            std::cos(circle_theta_) * circle_forward_.y +
            std::sin(circle_theta_) * circle_left_.y),
        circle_center_.z);

      const double error_xy = xyError(current_position, reference_position);
      const double scale = adaptiveCircleScale(error_xy);
      const double adaptive_omega = omega * scale;

      velocity_cmd = computeCircleVelocityCommand(
        reference_position,
        current_position,
        circle_theta_,
        adaptive_omega,
        dt);

      if (circle_theta_ < kTwoPi) {
        circle_theta_ += adaptive_omega * dt;
        circle_theta_ = std::min(circle_theta_, kTwoPi);
      }

      if (
        circle_theta_ >= kTwoPi &&
        stableAtXY(current_position, circle_start_position_, position_tolerance_, now_sec))
      {
        transitionTo(MissionState::RETURN_HOME, now_sec);
      }

      break;
    }

    case MissionState::RETURN_HOME:
    {
      if (!return_home_initialized_) {
        return_home_begin_ = current_position;
        return_home_initialized_ = true;
      }

      const double elapsed = now_sec - state_enter_sec_;

      reference_position = lineReference(
        return_home_begin_,
        home_position_,
        elapsed,
        line_speed_);

      velocity_cmd = computeMissionVelocityCommand(
        reference_position,
        current_position,
        dt);

      const double duration = lineDuration(
        return_home_begin_,
        home_position_,
        line_speed_);

      if (
        elapsed >= duration &&
        stableAtXY(current_position, home_position_, position_tolerance_, now_sec))
      {
        hold_position_ = home_position_;
        finishTask(now_sec);
      }

      break;
    }

    // 【新增】GO_TO_OFFSET 状态：从当前位置直线飞到偏移点
    case MissionState::GO_TO_OFFSET:
    {
      if (!offset_initialized_) {
        offset_begin_position_ = current_position;
        offset_initialized_ = true;
      }

      const double elapsed = now_sec - state_enter_sec_;

      reference_position = lineReference(
        offset_begin_position_,
        offset_position_,
        elapsed,
        line_speed_);

      velocity_cmd = computeMissionVelocityCommand(
        reference_position,
        current_position,
        dt);

      const double duration = lineDuration(
        offset_begin_position_,
        offset_position_,
        line_speed_);

      if (
        elapsed >= duration &&
        stableAtXY(current_position, offset_position_, position_tolerance_, now_sec))
      {
        // 到达偏移点后，按 finish_action 进入 FINAL_HOLD 或 HOLD
        finishTask(now_sec);
      }

      break;
    }

    case MissionState::FINAL_HOLD:
    {
      reference_position = hold_position_;

      velocity_cmd = computeMissionVelocityCommand(
        reference_position,
        current_position,
        dt);

      if ((now_sec - state_enter_sec_) >= final_hold_time_) {
        transitionTo(MissionState::LAND, now_sec);
      }

      break;
    }

    case MissionState::LAND:
    {
      reference_position = current_position;
      velocity_cmd = Vec3();

      if (mavros_->mode() != "AUTO.LAND") {
        if (serviceRequestAllowed(now_sec, last_mode_request_sec_)) {
          mavros_->requestLandMode();
        }
      }

      if (current_position.z <= landed_height_threshold_ && mavros_->armed()) {
        // 【新增】首次到达着陆高度时记录并打印落点误差
        if (!landed_logged_) {
          landed_logged_ = true;
          landed_position_ = current_position;

          const double dx = current_position.x - home_position_.x;
          const double dy = current_position.y - home_position_.y;
          const double xy_err = std::sqrt(dx * dx + dy * dy);

          RCLCPP_INFO(node_->get_logger(), "========================================");
          RCLCPP_INFO(node_->get_logger(), "  PASSIVE LANDING (AUTO.LAND) COMPLETE");
          RCLCPP_INFO(node_->get_logger(), "  Landed  : x=%.4f, y=%.4f, z=%.4f",
                      current_position.x, current_position.y, current_position.z);
          RCLCPP_INFO(node_->get_logger(), "  Home    : x=%.4f, y=%.4f, z=%.4f",
                      home_position_.x, home_position_.y, home_position_.z);
          RCLCPP_INFO(node_->get_logger(), "  XY Error: dx=%.4f, dy=%.4f, total=%.4f m",
                      dx, dy, xy_err);
          RCLCPP_INFO(node_->get_logger(), "========================================");
        }

        if (serviceRequestAllowed(now_sec, last_arm_request_sec_)) {
          mavros_->requestArm(false);
        }
      }

      break;
    }

    case MissionState::HOLD:
    {
      reference_position = hold_position_;

      velocity_cmd = computeMissionVelocityCommand(
        reference_position,
        current_position,
        dt);

      break;
    }
  }

  mavros_->publishReferenceAndPaths(reference_position, activeYaw());
  mavros_->publishVelocity(velocity_cmd, activeYaw());
}

void MissionManager::lockHomePoint(const Vec3 & current_position)
{
  home_position_ = current_position;
  hold_position_ = home_position_;
  locked_yaw_ = mavros_->currentYaw();
  home_locked_ = true;

  circle_center_ = home_position_;

  circle_forward_ = Vec3(
    std::cos(locked_yaw_),
    std::sin(locked_yaw_),
    0.0);

  circle_left_ = Vec3(
    -std::sin(locked_yaw_),
    std::cos(locked_yaw_),
    0.0);

  circle_start_position_ = Vec3(
    circle_center_.x + circle_radius_ * circle_forward_.x,
    circle_center_.y + circle_radius_ * circle_forward_.y,
    circle_center_.z);

  go_to_circle_start_initialized_ = false;
  return_home_initialized_ = false;

  // 【新增】计算偏移目标点（基于 Home 点水平偏移）
  offset_position_ = home_position_;
  offset_position_.x += offset_distance_x_;
  offset_position_.y += offset_distance_y_;

  generateSquareTargets();
  publishIdealMissionPath();

  pid_.reset();
  resetAltitudeController();

  RCLCPP_INFO(
    node_->get_logger(),
    "HOME LOCKED at current ENU position: x=%.3f, y=%.3f, z=%.3f",
    home_position_.x,
    home_position_.y,
    home_position_.z);

  RCLCPP_INFO(
    node_->get_logger(),
    "All tasks will hold this z height and yaw: z=%.3f, yaw=%.3f rad",
    home_position_.z,
    locked_yaw_);
}

void MissionManager::generateSquareTargets()
{
  square_targets_.clear();
  square_hold_flags_.clear();
  square_target_index_ = 0;
  square_point_arrive_sec_ = -1.0;
  last_square_target_advance_sec_ = -1.0e9;

  const double s = square_side_;

  const Vec3 p0(home_position_.x,     home_position_.y,     home_position_.z);
  const Vec3 p1(home_position_.x + s, home_position_.y,     home_position_.z);
  const Vec3 p2(home_position_.x + s, home_position_.y + s, home_position_.z);
  const Vec3 p3(home_position_.x,     home_position_.y + s, home_position_.z);

  appendSquareSegmentTargets(p0, p1, square_step_, true);
  appendSquareSegmentTargets(p1, p2, square_step_, true);
  appendSquareSegmentTargets(p2, p3, square_step_, true);
  appendSquareSegmentTargets(p3, p0, square_step_, true);
}

void MissionManager::appendSquareSegmentTargets(
  const Vec3 & start,
  const Vec3 & goal,
  double step,
  bool hold_at_goal)
{
  const Vec3 delta = goal - start;
  const double distance = norm3D(delta);

  if (distance < 1e-6) {
    square_targets_.push_back(goal);
    square_hold_flags_.push_back(hold_at_goal);
    return;
  }

  const double safe_step = std::max(0.05, step);
  const int count = std::max(1, static_cast<int>(std::ceil(distance / safe_step)));

  for (int i = 1; i <= count; ++i) {
    const double ratio = static_cast<double>(i) / static_cast<double>(count);
    square_targets_.push_back(start + delta * ratio);
    square_hold_flags_.push_back(i == count ? hold_at_goal : false);
  }
}

void MissionManager::finishTask(double now_sec)
{
  hold_position_ = home_position_;

  if (finish_action_ == "land") {
    transitionTo(MissionState::FINAL_HOLD, now_sec);
  } else {
    transitionTo(MissionState::HOLD, now_sec);
  }
}

double MissionManager::activeYaw() const
{
  if (home_locked_) {
    return locked_yaw_;
  }

  return mavros_->currentYaw();
}

void MissionManager::transitionTo(MissionState next_state, double now_sec)
{
  if (state_ == next_state) {
    return;
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "Mission transition: %s -> %s",
    stateName(state_),
    stateName(next_state));

  state_ = next_state;
  state_enter_sec_ = now_sec;
  stable_since_sec_ = -1.0;
  square_point_arrive_sec_ = -1.0;

  if (next_state == MissionState::SQUARE) {
    square_target_index_ = 0;
    last_square_target_advance_sec_ = -1.0e9;
  }

  if (next_state == MissionState::GO_TO_CIRCLE_START) {
    go_to_circle_start_initialized_ = false;
  }

  // 【新增】进入偏移直飞时重置起始标志
  if (next_state == MissionState::GO_TO_OFFSET) {
    offset_initialized_ = false;
  }

  if (next_state == MissionState::CIRCLE) {
    circle_theta_ = 0.0;
  }

  if (next_state == MissionState::RETURN_HOME) {
    return_home_initialized_ = false;
  }

  // 【新增】进入降落时重置落点记录标志
  if (next_state == MissionState::LAND) {
    landed_logged_ = false;
    landed_position_ = Vec3();
  }

  configureControlProfile(next_state);
}

void MissionManager::configureControlProfile(MissionState state)
{
  switch (state) {
    case MissionState::SQUARE:
    {
      max_vxy_ = square_max_vxy_;
      pid_.configure(square_x_config_, square_y_config_, z_config_);
      break;
    }

    case MissionState::GO_TO_CIRCLE_START:
    case MissionState::CIRCLE:
    case MissionState::RETURN_HOME:
    {
      max_vxy_ = circle_max_vxy_;
      pid_.configure(circle_x_config_, circle_y_config_, z_config_);
      break;
    }

    // 【新增】GO_TO_OFFSET 使用悬停 PID（保守稳定）
    case MissionState::GO_TO_OFFSET:
    case MissionState::HOVER:
    case MissionState::FINAL_HOLD:
    case MissionState::HOLD:
    case MissionState::WAIT_FOR_FCU:
    case MissionState::WAIT_FOR_MANUAL_OFFBOARD:
    case MissionState::LAND:
    default:
    {
      max_vxy_ = hover_max_vxy_;
      pid_.configure(hover_x_config_, hover_y_config_, z_config_);
      break;
    }
  }

  pid_.reset();
  resetAltitudeController();
}

const char * MissionManager::stateName(MissionState state) const
{
  switch (state) {
    case MissionState::WAIT_FOR_FCU:
      return "WAIT_FOR_FCU";
    case MissionState::WAIT_FOR_MANUAL_OFFBOARD:
      return "WAIT_FOR_MANUAL_OFFBOARD";
    case MissionState::HOVER:
      return "HOVER";
    case MissionState::SQUARE:
      return "SQUARE";
    case MissionState::GO_TO_CIRCLE_START:
      return "GO_TO_CIRCLE_START";
    case MissionState::CIRCLE:
      return "CIRCLE";
    case MissionState::RETURN_HOME:
      return "RETURN_HOME";
    case MissionState::GO_TO_OFFSET:      // 【新增】
      return "GO_TO_OFFSET";
    case MissionState::FINAL_HOLD:
      return "FINAL_HOLD";
    case MissionState::LAND:
      return "LAND";
    case MissionState::HOLD:
      return "HOLD";
    default:
      return "UNKNOWN";
  }
}

bool MissionManager::stableAt(
  const Vec3 & current_position,
  const Vec3 & target_position,
  double now_sec)
{
  const double error = distance3D(current_position, target_position);

  if (error > position_tolerance_) {
    stable_since_sec_ = -1.0;
    return false;
  }

  if (stable_since_sec_ < 0.0) {
    stable_since_sec_ = now_sec;
    return false;
  }

  return (now_sec - stable_since_sec_) >= settle_time_;
}

bool MissionManager::stableAtXY(
  const Vec3 & current_position,
  const Vec3 & target_position,
  double tolerance,
  double now_sec)
{
  const double error = xyError(current_position, target_position);

  if (error > tolerance) {
    stable_since_sec_ = -1.0;
    return false;
  }

  if (stable_since_sec_ < 0.0) {
    stable_since_sec_ = now_sec;
    return false;
  }

  return (now_sec - stable_since_sec_) >= settle_time_;
}

bool MissionManager::serviceRequestAllowed(
  double now_sec,
  double & last_request_sec) const
{
  if ((now_sec - last_request_sec) >= service_request_period_) {
    last_request_sec = now_sec;
    return true;
  }

  return false;
}

Vec3 MissionManager::computeMissionVelocityCommand(
  const Vec3 & reference_position,
  const Vec3 & current_position,
  double dt)
{
  Vec3 velocity_cmd = pid_.computeVelocityCommand(
    reference_position,
    current_position,
    dt);

  velocity_cmd.z = computeAltitudeVelocity(
    reference_position.z,
    current_position.z,
    dt);

  return clampVelocityCommand(velocity_cmd);
}

Vec3 MissionManager::computeCircleVelocityCommand(
  const Vec3 & reference_position,
  const Vec3 & current_position,
  double theta,
  double omega,
  double dt)
{
  Vec3 velocity_cmd = pid_.computeVelocityCommand(
    reference_position,
    current_position,
    dt);

  const Vec3 feedforward_velocity(
    circle_radius_ * omega *
      (-std::sin(theta) * circle_forward_.x + std::cos(theta) * circle_left_.x),
    circle_radius_ * omega *
      (-std::sin(theta) * circle_forward_.y + std::cos(theta) * circle_left_.y),
    0.0);

  velocity_cmd.x += feedforward_velocity.x;
  velocity_cmd.y += feedforward_velocity.y;

  velocity_cmd.z = computeAltitudeVelocity(
    reference_position.z,
    current_position.z,
    dt);

  return clampVelocityCommand(velocity_cmd);
}

Vec3 MissionManager::lineReference(
  const Vec3 & start,
  const Vec3 & goal,
  double elapsed,
  double speed) const
{
  const Vec3 delta = goal - start;
  const double distance = norm3D(delta);

  if (distance < 1e-6) {
    return goal;
  }

  const double safe_speed = std::max(0.05, speed);
  const double ratio = clampValue(elapsed * safe_speed / distance, 0.0, 1.0);

  return start + delta * ratio;
}

double MissionManager::lineDuration(
  const Vec3 & start,
  const Vec3 & goal,
  double speed) const
{
  const double safe_speed = std::max(0.05, speed);
  return distance3D(start, goal) / safe_speed;
}

double MissionManager::computeAltitudeVelocity(
  double target_z,
  double current_z,
  double dt)
{
  if (dt <= 1e-6 || !std::isfinite(dt)) {
    return 0.0;
  }

  double error = target_z - current_z;

  if (std::abs(error) < altitude_deadband_) {
    error = 0.0;
    altitude_integral_ *= 0.98;
  }

  altitude_integral_ += error * dt;
  altitude_integral_ = clampValue(
    altitude_integral_,
    -altitude_integral_limit_,
    altitude_integral_limit_);

  double derivative = 0.0;

  if (!altitude_first_update_) {
    derivative = (error - altitude_previous_error_) / dt;
  }

  altitude_derivative_filtered_ =
    altitude_derivative_alpha_ * derivative +
    (1.0 - altitude_derivative_alpha_) * altitude_derivative_filtered_;

  altitude_previous_error_ = error;
  altitude_first_update_ = false;

  const double vz =
    altitude_kp_ * error +
    altitude_ki_ * altitude_integral_ +
    altitude_kd_ * altitude_derivative_filtered_;

  return clampValue(vz, -max_vz_down_, max_vz_up_);
}

void MissionManager::resetAltitudeController()
{
  altitude_integral_ = 0.0;
  altitude_previous_error_ = 0.0;
  altitude_derivative_filtered_ = 0.0;
  altitude_first_update_ = true;
}

void MissionManager::publishIdealMissionPath()
{
  std::vector<Vec3> points;

  const double line_step = 0.05;
  const double circle_step_angle = 0.03;

  points.push_back(home_position_);

  if (mission_phase_ == "square_only" || mission_phase_ == "full") {
    const double s = square_side_;

    const Vec3 p0(home_position_.x,     home_position_.y,     home_position_.z);
    const Vec3 p1(home_position_.x + s, home_position_.y,     home_position_.z);
    const Vec3 p2(home_position_.x + s, home_position_.y + s, home_position_.z);
    const Vec3 p3(home_position_.x,     home_position_.y + s, home_position_.z);

    appendLineSamples(points, p0, p1, line_step);
    appendLineSamples(points, p1, p2, line_step);
    appendLineSamples(points, p2, p3, line_step);
    appendLineSamples(points, p3, p0, line_step);
  }

  if (mission_phase_ == "circle_only" || mission_phase_ == "full") {
    appendLineSamples(points, home_position_, circle_start_position_, line_step);
    appendCircleSamples(points, circle_step_angle);
    appendLineSamples(points, circle_start_position_, home_position_, line_step);
  }

  mavros_->setTargetPath(points);
}

void MissionManager::appendLineSamples(
  std::vector<Vec3> & points,
  const Vec3 & start,
  const Vec3 & goal,
  double step) const
{
  const double distance = distance3D(start, goal);

  if (distance < 1e-6) {
    points.push_back(goal);
    return;
  }

  const int count =
    std::max(1, static_cast<int>(std::ceil(distance / std::max(0.01, step))));

  for (int i = 1; i <= count; ++i) {
    const double ratio = static_cast<double>(i) / static_cast<double>(count);
    points.push_back(start + (goal - start) * ratio);
  }
}

void MissionManager::appendCircleSamples(
  std::vector<Vec3> & points,
  double step_angle) const
{
  const int count =
    std::max(8, static_cast<int>(std::ceil(kTwoPi / std::max(0.01, step_angle))));

  for (int i = 1; i <= count; ++i) {
    const double theta =
      static_cast<double>(i) / static_cast<double>(count) * kTwoPi;

    points.push_back(Vec3(
      circle_center_.x +
        circle_radius_ * (
          std::cos(theta) * circle_forward_.x +
          std::sin(theta) * circle_left_.x),
      circle_center_.y +
        circle_radius_ * (
          std::cos(theta) * circle_forward_.y +
          std::sin(theta) * circle_left_.y),
      circle_center_.z));
  }
}

Vec3 MissionManager::clampVelocityCommand(const Vec3 & velocity) const
{
  return Vec3(
    clampValue(velocity.x, -max_vxy_, max_vxy_),
    clampValue(velocity.y, -max_vxy_, max_vxy_),
    clampValue(velocity.z, -max_vz_down_, max_vz_up_));
}

double MissionManager::xyError(
  const Vec3 & a,
  const Vec3 & b) const
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return std::sqrt(dx * dx + dy * dy);
}

double MissionManager::adaptiveCircleScale(double error_xy) const
{
  if (error_xy <= circle_error_slow_) {
    return 1.0;
  }

  if (error_xy <= circle_error_very_slow_) {
    return circle_slow_scale_;
  }

  return circle_min_scale_;
}

}  // namespace uav_pid_mavros