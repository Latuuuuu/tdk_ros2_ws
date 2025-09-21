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
    MissionManager() : Node("mission_manager"), current_mission_(-1) {
        // 監聽任務完成狀態
        mission_starter_sub_ = this->create_subscription<std_msgs::msg::Int32>("/mission_starter", 10, std::bind(&MissionManager::mission_starter_callback, this, std::placeholders::_1));
        mission_status_sub_ = this->create_subscription<std_msgs::msg::Int32>("/mission_status", 10, std::bind(&MissionManager::mission_status_callback, this, std::placeholders::_1));
        mission_command_pub_ = this->create_publisher<std_msgs::msg::Int32>("/mission_command", 10);
        // 啟動第一個任務
        start_next_mission();

        timer_ = this->create_wall_timer(std::chrono::milliseconds(50),std::bind(&MissionManager::send_mission, this));

        RCLCPP_INFO(this->get_logger(), "Mission Manager started.");
    }

private:
    void start_next_mission() {
        if (current_mission_ == -1) {
            RCLCPP_INFO(this->get_logger(), "Resetting Missions...");
            // current_mission_ = 1;
        } else if (current_mission_ == 1) {
            RCLCPP_INFO(this->get_logger(), "Starting Mission Two...");
            current_mission_ = 2;
        } else if (current_mission_ == 2) {
            RCLCPP_INFO(this->get_logger(), "Starting Mission Four...");
            current_mission_ = 4;
        } else if (current_mission_ == 0) {
            RCLCPP_INFO(this->get_logger(), "Not Starting Mission...");
        } else {
            RCLCPP_INFO(this->get_logger(), "All missions completed!");
            rclcpp::shutdown();
        }
    }

    void mission_status_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        if (msg->data == current_mission_) {
            start_next_mission();
        }
    }

    void send_mission() {
        if (current_mission_ == 1) {
            std_msgs::msg::Int32 cmd;
            cmd.data = 1;
            mission_command_pub_->publish(cmd);
        } else if (current_mission_ == 2) {
            std_msgs::msg::Int32 cmd;
            cmd.data = 2;
            mission_command_pub_->publish(cmd);
        }else if (current_mission_ == 4) {
            std_msgs::msg::Int32 cmd;
            cmd.data = 4;
            mission_command_pub_->publish(cmd);
        }else if (current_mission_ == -1) {
            std_msgs::msg::Int32 cmd;
            cmd.data = -1;
            mission_command_pub_->publish(cmd);
        }else {
            std_msgs::msg::Int32 cmd;
            cmd.data = 0;
            mission_command_pub_->publish(cmd);
        }
    }

    void mission_starter_callback(const std_msgs::msg::Int32::SharedPtr msg){
        current_mission_=msg->data;
    }

    int current_mission_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr mission_status_sub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr mission_command_pub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr mission_starter_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};