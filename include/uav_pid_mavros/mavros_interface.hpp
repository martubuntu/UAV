#pragma once

#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

#include "mavros_msgs/msg/position_target.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/set_mode.hpp"

#include "uav_pid_mavros/types.hpp"
#include <fstream>

namespace uav_pid_mavros
{

class MavrosInterface
{
public:
  explicit MavrosInterface(const rclcpp::Node::SharedPtr & node);

  bool connected() const;
  bool armed() const;
  bool hasLocalPose() const;
  std::string mode() const;

  Vec3 currentPosition() const;
  double currentYaw() const;

  void publishVelocity(const Vec3 & velocity_enu, double yaw_enu_rad);
  void publishReferenceAndPaths(const Vec3 & reference_position, double yaw_enu_rad);
  void setTargetPath(const std::vector<Vec3> & target_points);


  void requestArm(bool arm);
  void requestMode(const std::string & mode);
  void requestOffboardMode();
  void requestLandMode();

private:
  void stateCallback(const mavros_msgs::msg::State::SharedPtr msg);
  void localPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  std::string frameId() const;

  void appendPathPoint(
    nav_msgs::msg::Path & path,
    const geometry_msgs::msg::PoseStamped & pose);

  void writeTrajectoryLog(
  const rclcpp::Time & stamp,
  const Vec3 & target_position,
  double target_yaw,
  const Vec3 & actual_position,
  double actual_yaw,
  bool has_actual);

  rclcpp::Node::SharedPtr node_;

  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr local_pose_sub_;

  rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr setpoint_raw_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr reference_pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr target_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr actual_path_pub_;

  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;

  mavros_msgs::msg::State current_state_;
  geometry_msgs::msg::PoseStamped current_local_pose_;

  bool has_state_{false};
  bool has_local_pose_{false};

  bool trajectory_log_enabled_{false};
  std::ofstream trajectory_log_file_;
  std::size_t trajectory_log_count_{0};

  nav_msgs::msg::Path target_path_;
  nav_msgs::msg::Path actual_path_;

  bool has_last_target_point_{false};
  bool has_last_actual_point_{false};
  Vec3 last_target_point_;
  Vec3 last_actual_point_;

  std::string rviz_frame_id_{"map"};
  double path_min_step_{0.02};
  std::size_t path_max_size_{5000};
};

}  // namespace uav_pid_mavros
