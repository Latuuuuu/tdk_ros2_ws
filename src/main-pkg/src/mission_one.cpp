#include "main-pkg/mission_one.hpp"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MissionOne>());
    rclcpp::shutdown();
    return 0;
}