#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "uav_pid_mavros/mission_manager.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("uav_pid_mission");

  auto mission_manager =
    std::make_shared<uav_pid_mavros::MissionManager>(node);

  mission_manager->start();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}