#include "main-pkg/mission_two.hpp"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MissionTwo>());
    rclcpp::shutdown();
    return 0;
}