#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "main-pkg/mission_two.hpp"
#include "main-pkg/mission_four.hpp"

class MissionManager : public rclcpp::Node {
public:
    MissionManager() : Node("mission_manager"), current_mission_(0) {
        // 監聽任務完成狀態
        mission_status_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/mission_status", 10,
            std::bind(&MissionManager::mission_status_callback, this, std::placeholders::_1));
        
        // 啟動第一個任務
        start_next_mission();
        
        RCLCPP_INFO(this->get_logger(), "Mission Manager started.");
    }

private:
    void start_next_mission() {
        if (current_mission_ == 0) {
            RCLCPP_INFO(this->get_logger(), "Starting Mission Two...");
            mission_two_ = std::make_shared<MissionTwo>();
            current_mission_ = 2;
        } else if (current_mission_ == 2) {
            RCLCPP_INFO(this->get_logger(), "Starting Mission Four...");
            mission_four_ = std::make_shared<MissionFour>();
            current_mission_ = 4;
        } else {
            RCLCPP_INFO(this->get_logger(), "All missions completed!");
            rclcpp::shutdown();
        }
    }

    void mission_status_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        if (msg->data == current_mission_) {
            RCLCPP_INFO(this->get_logger(), "Mission %d completed.", current_mission_);
            // 清理當前任務
            if (current_mission_ == 2) {
                mission_two_.reset();
            } else if (current_mission_ == 4) {
                mission_four_.reset();
            }
            // 啟動下一個任務
            start_next_mission();
        }
    }

    int current_mission_;
    std::shared_ptr<MissionTwo> mission_two_;
    std::shared_ptr<MissionFour> mission_four_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr mission_status_sub_;
};