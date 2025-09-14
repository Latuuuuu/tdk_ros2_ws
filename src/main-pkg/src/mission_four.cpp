#include "main-pkg/mission_four.hpp"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MissionFour>());
    rclcpp::shutdown();
    return 0;
}