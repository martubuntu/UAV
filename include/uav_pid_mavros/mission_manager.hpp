#ifndef UAV_PID_MAVROS_MISSION_MANAGER_HPP_
#define UAV_PID_MAVROS_MISSION_MANAGER_HPP_

#include <memory>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "uav_pid_mavros/mavros_interface.hpp"
#include "uav_pid_mavros/pid_controller.hpp"

namespace uav_pid_mavros {

// 任务状态枚举
enum class MissionState {
  WAIT_FOR_FCU,
  WAIT_FOR_MANUAL_OFFBOARD,
  HOVER,
  SQUARE,
  GO_TO_CIRCLE_START,
  CIRCLE,
  RETURN_HOME,
  GO_TO_OFFSET,    // 【新增】偏移直飞状态
  FINAL_HOLD,
  LAND,
  HOLD
};

// PID 单轴配置（假设定义在此或包含的头文件中）
struct AxisPidConfig {
  double kp;
  double ki;
  double kd;
  double integral_limit;
  double output_limit;
};

class MissionManager {
public:
  explicit MissionManager(const rclcpp::Node::SharedPtr & node);
  void start();

private:
  void update();
  void lockHomePoint(const Vec3 & current_position);
  void generateSquareTargets();
  void appendSquareSegmentTargets(
    const Vec3 & start,
    const Vec3 & goal,
    double step,
    bool hold_at_goal);
  void finishTask(double now_sec);
  double activeYaw() const;
  void transitionTo(MissionState next_state, double now_sec);
  void configureControlProfile(MissionState state);
  const char * stateName(MissionState state) const;
  bool stableAt(
    const Vec3 & current_position,
    const Vec3 & target_position,
    double now_sec);
  bool stableAtXY(
    const Vec3 & current_position,
    const Vec3 & target_position,
    double tolerance,
    double now_sec);
  bool serviceRequestAllowed(
    double now_sec,
    double & last_request_sec) const;
  Vec3 computeMissionVelocityCommand(
    const Vec3 & reference_position,
    const Vec3 & current_position,
    double dt);
  Vec3 computeCircleVelocityCommand(
    const Vec3 & reference_position,
    const Vec3 & current_position,
    double theta,
    double omega,
    double dt);
  Vec3 lineReference(
    const Vec3 & start,
    const Vec3 & goal,
    double elapsed,
    double speed) const;
  double lineDuration(
    const Vec3 & start,
    const Vec3 & goal,
    double speed) const;
  double computeAltitudeVelocity(
    double target_z,
    double current_z,
    double dt);
  void resetAltitudeController();
  void publishIdealMissionPath();
  void appendLineSamples(
    std::vector<Vec3> & points,
    const Vec3 & start,
    const Vec3 & goal,
    double step) const;
  void appendCircleSamples(
    std::vector<Vec3> & points,
    double step_angle) const;
  Vec3 clampVelocityCommand(const Vec3 & velocity) const;
  double xyError(const Vec3 & a, const Vec3 & b) const;
  double adaptiveCircleScale(double error_xy) const;

  // ROS 节点与定时器
  rclcpp::Node::SharedPtr node_;
  rclcpp::TimerBase::SharedPtr timer_;

  // 任务参数
  double control_rate_hz_;
  double hover_time_;
  double pre_task_hold_time_;
  double final_hold_time_;
  double circle_radius_;
  double circle_speed_;
  double circle_error_slow_;
  double circle_error_very_slow_;
  double circle_slow_scale_;
  double circle_min_scale_;
  double line_speed_;
  double square_side_;
  double square_hold_time_;
  double square_step_;
  double square_target_tolerance_;
  double square_target_update_period_;
  double position_tolerance_;
  double settle_time_;
  double landed_height_threshold_;
  double hover_max_vxy_;
  double square_max_vxy_;
  double circle_max_vxy_;
  double max_vxy_;
  double max_vz_up_;
  double max_vz_down_;
  double altitude_kp_;
  double altitude_ki_;
  double altitude_kd_;
  double altitude_deadband_;
  double altitude_integral_limit_;
  double altitude_derivative_alpha_;
  bool auto_offboard_;
  bool auto_arm_;
  std::string mission_phase_;
  std::string finish_action_;
  double service_request_period_;

  // 【新增】偏移降落参数
  double offset_distance_x_ = 3.0;
  double offset_distance_y_ = 0.0;

  // PID 配置
  AxisPidConfig hover_x_config_;
  AxisPidConfig hover_y_config_;
  AxisPidConfig square_x_config_;
  AxisPidConfig square_y_config_;
  AxisPidConfig circle_x_config_;
  AxisPidConfig circle_y_config_;
  AxisPidConfig z_config_;

  // 子系统
  std::unique_ptr<MavrosInterface> mavros_;
  PidController pid_;

  // 时间记录
  double last_update_sec_;
  double state_enter_sec_;
  MissionState state_ = MissionState::WAIT_FOR_FCU;

  // Home 点与坐标系
  Vec3 home_position_;
  Vec3 hold_position_;
  bool home_locked_ = false;
  double locked_yaw_ = 0.0;

  // 圆形轨迹坐标系
  Vec3 circle_center_;
  Vec3 circle_forward_;
  Vec3 circle_left_;
  Vec3 circle_start_position_;
  bool go_to_circle_start_initialized_ = false;
  Vec3 go_to_circle_start_begin_;
  bool return_home_initialized_ = false;
  Vec3 return_home_begin_;
  double circle_theta_ = 0.0;

  // 【新增】偏移直飞状态变量
  Vec3 offset_position_;
  Vec3 offset_begin_position_;
  bool offset_initialized_ = false;

  // 正方形航点
  std::vector<Vec3> square_targets_;
  std::vector<bool> square_hold_flags_;
  std::size_t square_target_index_ = 0;
  double square_point_arrive_sec_ = -1.0;
  double last_square_target_advance_sec_ = -1.0e9;

  // 稳定判定与服务请求时间
  double stable_since_sec_ = -1.0;
  double last_mode_request_sec_ = -1.0e9;
  double last_arm_request_sec_ = -1.0e9;

  // 高度 PID 内部状态
  double altitude_integral_ = 0.0;
  double altitude_previous_error_ = 0.0;
  double altitude_derivative_filtered_ = 0.0;
  bool altitude_first_update_ = true;

  // 【新增】被动降落落点记录
  bool landed_logged_ = false;
  Vec3 landed_position_;
};

} // namespace uav_pid_mavros

#endif