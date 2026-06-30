#include "uav_pid_mavros/mavros_interface.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <utility>

namespace uav_pid_mavros
{

MavrosInterface::MavrosInterface(const rclcpp::Node::SharedPtr & node)
: node_(node)
{
  const std::string state_topic =
    node_->declare_parameter<std::string>("mavros_state_topic", "/mavros/state");

  const std::string local_pose_topic =
    node_->declare_parameter<std::string>("mavros_local_pose_topic", "/mavros/local_position/pose");

  const std::string raw_setpoint_topic =
    node_->declare_parameter<std::string>(
    "mavros_raw_setpoint_topic",
    "/mavros/setpoint_raw/local");

  const std::string arming_service =
    node_->declare_parameter<std::string>("mavros_arming_service", "/mavros/cmd/arming");

  const std::string set_mode_service =
    node_->declare_parameter<std::string>("mavros_set_mode_service", "/mavros/set_mode");

  rviz_frame_id_ =
    node_->declare_parameter<std::string>("rviz_frame_id", "map");

  path_min_step_ =
    node_->declare_parameter<double>("path_min_step", 0.02);

  path_max_size_ =
    static_cast<std::size_t>(node_->declare_parameter<int>("path_max_size", 5000));

  state_sub_ = node_->create_subscription<mavros_msgs::msg::State>(
    state_topic,
    rclcpp::QoS(10),
    std::bind(&MavrosInterface::stateCallback, this, std::placeholders::_1));

  local_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    local_pose_topic,
    rclcpp::SensorDataQoS(),
    std::bind(&MavrosInterface::localPoseCallback, this, std::placeholders::_1));

  setpoint_raw_pub_ = node_->create_publisher<mavros_msgs::msg::PositionTarget>(
    raw_setpoint_topic,
    rclcpp::QoS(10));

  reference_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/uav_pid_mavros/reference_pose",
    rclcpp::QoS(10));

  target_path_pub_ = node_->create_publisher<nav_msgs::msg::Path>(
    "/uav_pid_mavros/target_path",
    rclcpp::QoS(10));

  actual_path_pub_ = node_->create_publisher<nav_msgs::msg::Path>(
    "/uav_pid_mavros/actual_path",
    rclcpp::QoS(10));

  arming_client_ = node_->create_client<mavros_msgs::srv::CommandBool>(arming_service);
  set_mode_client_ = node_->create_client<mavros_msgs::srv::SetMode>(set_mode_service);

  target_path_.header.frame_id = frameId();
  actual_path_.header.frame_id = frameId();
}

bool MavrosInterface::connected() const
{
  return has_state_ && current_state_.connected;
}

bool MavrosInterface::armed() const
{
  return has_state_ && current_state_.armed;
}

bool MavrosInterface::hasLocalPose() const
{
  return has_local_pose_;
}

std::string MavrosInterface::mode() const
{
  if (!has_state_) {
    return "";
  }
  return current_state_.mode;
}

Vec3 MavrosInterface::currentPosition() const
{
  if (!has_local_pose_) {
    return Vec3();
  }

  return Vec3(
    current_local_pose_.pose.position.x,
    current_local_pose_.pose.position.y,
    current_local_pose_.pose.position.z);
}

double MavrosInterface::currentYaw() const
{
  if (!has_local_pose_) {
    return 0.0;
  }

  const auto & q = current_local_pose_.pose.orientation;
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

void MavrosInterface::publishVelocity(const Vec3 & velocity_enu, double yaw_enu_rad)
{
  mavros_msgs::msg::PositionTarget cmd;
  cmd.header.stamp = node_->now();
  cmd.header.frame_id = frameId();
  cmd.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;

  cmd.type_mask =
    mavros_msgs::msg::PositionTarget::IGNORE_PX |
    mavros_msgs::msg::PositionTarget::IGNORE_PY |
    mavros_msgs::msg::PositionTarget::IGNORE_PZ |
    mavros_msgs::msg::PositionTarget::IGNORE_AFX |
    mavros_msgs::msg::PositionTarget::IGNORE_AFY |
    mavros_msgs::msg::PositionTarget::IGNORE_AFZ |
    mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE;

  cmd.velocity.x = velocity_enu.x;
  cmd.velocity.y = velocity_enu.y;
  cmd.velocity.z = velocity_enu.z;
  cmd.yaw = yaw_enu_rad;

  setpoint_raw_pub_->publish(cmd);
}

void MavrosInterface::publishReferenceAndPaths(
  const Vec3 & reference_position,
  double yaw_enu_rad)
{
  const auto stamp = node_->now();
  const std::string frame = frameId();

  geometry_msgs::msg::PoseStamped reference_pose;
  reference_pose.header.stamp = stamp;
  reference_pose.header.frame_id = frame;
  reference_pose.pose.position.x = reference_position.x;
  reference_pose.pose.position.y = reference_position.y;
  reference_pose.pose.position.z = reference_position.z;
  reference_pose.pose.orientation.z = std::sin(yaw_enu_rad * 0.5);
  reference_pose.pose.orientation.w = std::cos(yaw_enu_rad * 0.5);

  reference_pose_pub_->publish(reference_pose);

  if (
    !has_last_target_point_ ||
    distance3D(reference_position, last_target_point_) >= path_min_step_)
  {
    appendPathPoint(target_path_, reference_pose);
    last_target_point_ = reference_position;
    has_last_target_point_ = true;
  }

  Vec3 actual_position{};
  [[maybe_unused]] double actual_yaw = 0.0;
  [[maybe_unused]] bool has_actual_position = false;

  if (has_local_pose_) {
    geometry_msgs::msg::PoseStamped actual_pose = current_local_pose_;
    actual_pose.header.stamp = stamp;
    actual_pose.header.frame_id = frame;

    actual_position = Vec3(
      actual_pose.pose.position.x,
      actual_pose.pose.position.y,
      actual_pose.pose.position.z);

    actual_yaw = currentYaw();
    has_actual_position = true;

    if (
      !has_last_actual_point_ ||
      distance3D(actual_position, last_actual_point_) >= path_min_step_)
    {
      appendPathPoint(actual_path_, actual_pose);
      last_actual_point_ = actual_position;
      has_last_actual_point_ = true;
    }
  }

  target_path_.header.stamp = stamp;
  target_path_.header.frame_id = frame;
  actual_path_.header.stamp = stamp;
  actual_path_.header.frame_id = frame;

  target_path_pub_->publish(target_path_);
  actual_path_pub_->publish(actual_path_);
}


void MavrosInterface::setTargetPath(const std::vector<Vec3> & target_points)
{
  const auto stamp = node_->now();
  const std::string frame = frameId();

  target_path_.header.stamp = stamp;
  target_path_.header.frame_id = frame;
  target_path_.poses.clear();
  has_last_target_point_ = false;

  for (const auto & point : target_points) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = stamp;
    pose.header.frame_id = frame;
    pose.pose.position.x = point.x;
    pose.pose.position.y = point.y;
    pose.pose.position.z = point.z;
    pose.pose.orientation.w = 1.0;
    appendPathPoint(target_path_, pose);
  }

  if (!target_points.empty()) {
    last_target_point_ = target_points.back();
    has_last_target_point_ = true;
  }

  target_path_pub_->publish(target_path_);
}

void MavrosInterface::requestArm(bool arm)
{
  if (!arming_client_->service_is_ready()) {
    RCLCPP_WARN(node_->get_logger(), "Arming service is not ready.");
    return;
  }

  auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
  request->value = arm;

  arming_client_->async_send_request(
    request,
    [this, arm](rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future) {
      const auto response = future.get();
      if (response->success) {
        RCLCPP_INFO(
          node_->get_logger(),
          "Arm request accepted: %s",
          arm ? "ARM" : "DISARM");
      } else {
        RCLCPP_WARN(
          node_->get_logger(),
          "Arm request rejected: %s, result=%u",
          arm ? "ARM" : "DISARM",
          response->result);
      }
    });
}

void MavrosInterface::requestMode(const std::string & mode)
{
  if (!set_mode_client_->service_is_ready()) {
    RCLCPP_WARN(node_->get_logger(), "SetMode service is not ready.");
    return;
  }

  auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
  request->base_mode = 0;
  request->custom_mode = mode;

  set_mode_client_->async_send_request(
    request,
    [this, mode](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future) {
      const auto response = future.get();
      if (response->mode_sent) {
        RCLCPP_INFO(node_->get_logger(), "Mode request sent: %s", mode.c_str());
      } else {
        RCLCPP_WARN(node_->get_logger(), "Mode request failed: %s", mode.c_str());
      }
    });
}

void MavrosInterface::requestOffboardMode()
{
  requestMode("OFFBOARD");
}

void MavrosInterface::requestLandMode()
{
  requestMode("AUTO.LAND");
}

void MavrosInterface::stateCallback(const mavros_msgs::msg::State::SharedPtr msg)
{
  current_state_ = *msg;
  has_state_ = true;
}

void MavrosInterface::localPoseCallback(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  current_local_pose_ = *msg;
  has_local_pose_ = true;
}

std::string MavrosInterface::frameId() const
{
  if (!rviz_frame_id_.empty()) {
    return rviz_frame_id_;
  }

  if (has_local_pose_ && !current_local_pose_.header.frame_id.empty()) {
    return current_local_pose_.header.frame_id;
  }

  return "map";
}

void MavrosInterface::appendPathPoint(
  nav_msgs::msg::Path & path,
  const geometry_msgs::msg::PoseStamped & pose)
{
  path.poses.push_back(pose);

  if (path.poses.size() > path_max_size_) {
    const std::size_t remove_count = path.poses.size() - path_max_size_;
    path.poses.erase(path.poses.begin(), path.poses.begin() + remove_count);
  }
}

}  // namespace uav_pid_mavros
