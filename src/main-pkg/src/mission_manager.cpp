#include "main-pkg/mission_manager.hpp"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MissionManager>());
    rclcpp::shutdown();
    return 0;
}