#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "interfaces/srv/goal_point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class MissionTwo : public rclcpp::Node {
public:
    MissionTwo() : Node("mission_2") {
        // 建立目標點的 client
        goal_client_ = this->create_client<interfaces::srv::GoalPoint>("/goal");

        // 初始化目標點序列
        initialize_goals();

        // 計時器，用於檢查目標完成狀態並發送下一個目標
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100),std::bind(&MissionTwo::check_and_send_goal, this)
        );

        RCLCPP_INFO(this->get_logger(), "MissionTwo started.");
    }

private:
    void initialize_goals() {
        // 初始化目標點序列
        geometry_msgs::msg::PoseStamped goal1;
        goal1.pose.position.x = 10.0;
        goal1.pose.position.y = 0.0;
        goal1.pose.orientation.z = 0.0; // 無旋轉
        goals_.push_back(goal1);

        geometry_msgs::msg::PoseStamped goal2;
        goal2.pose.position.x = 20.0;
        goal2.pose.position.y = 10.0;
        goal2.pose.orientation.z = 3.14; // 無旋轉
        goals_.push_back(goal2);

        geometry_msgs::msg::PoseStamped goal3;
        goal3.pose.position.x = 0.0;
        goal3.pose.position.y = 0.0;
        goal3.pose.orientation.z = 0.0; // 無旋轉
        goals_.push_back(goal3);
    }

    void check_and_send_goal() {
        // 如果目前沒有目標，或目標序列已完成，則退出
        if (goals_.empty()) {
            RCLCPP_INFO(this->get_logger(), "All goals completed.");
            rclcpp::shutdown();
            return;
        }

        // 如果正在等待服務回應，則不發送新目標
        if (waiting_for_response_) {
            return;
        }

        // 發送下一個目標
        auto request = std::make_shared<interfaces::srv::GoalPoint::Request>();
        request->goal = goals_.front();

        // 呼叫服務
        auto future = goal_client_->async_send_request(
            request,
            std::bind(&MissionTwo::goal_response_callback, this, std::placeholders::_1)
        );

        waiting_for_response_ = true;
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Sent goal: x=%.2f, y=%.2f", request->goal.pose.position.x, request->goal.pose.position.y);
    }

    void goal_response_callback(rclcpp::Client<interfaces::srv::GoalPoint>::SharedFuture future) {
        auto response = future.get();
        if (response->status) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Goal reached.");
            goals_.erase(goals_.begin()); // 移除已完成的目標
        } else {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Goal not reached yet.");
        }
        waiting_for_response_ = false;
    }

    // 成員變數
    rclcpp::Client<interfaces::srv::GoalPoint>::SharedPtr goal_client_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<geometry_msgs::msg::PoseStamped> goals_;
    bool waiting_for_response_{false};
};