#include "rclcpp/rclcpp.hpp"
#include "main-pkg/chassis_pilot.hpp"

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ChassisPilot>());
  rclcpp::shutdown();
  return 0;
}