#include "rclcpp/rclcpp.hpp"
#include "main-pkg/sim_lower.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimLower>());
  rclcpp::shutdown();
  return 0;
}
