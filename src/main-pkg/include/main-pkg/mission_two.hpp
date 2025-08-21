#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "interfaces/srv/goal_point.hpp"
#include "interfaces/srv/menu.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class MissionTwo : public rclcpp::Node {
public:
    MissionTwo() : Node("mission_2") {
        // 建立目標點的 client
        goal_client_ = this->create_client<interfaces::srv::GoalPoint>("/goal");
        menu_client_ = this->create_client<interfaces::srv::Menu>("/start");

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
        Goal goal1;
        goal1.type = 0;
        goal1.pose.pose.position.x = 10.0;
        goal1.pose.pose.position.y = 0.0;
        goal1.pose.pose.orientation.z = 0.0;
        goal1.max_linear_speed = 1.0;
        goal1.max_angular_speed = 0.0;
        goals_.push_back(goal1);

        Goal goal2;
        goal2.type = 1;
        goal2.start = true;
        goals_.push_back(goal2);

        Goal goal3;
        goal3.type = 0;
        goal3.pose.pose.position.x = 20.0;
        goal3.pose.pose.position.y = 10.0;
        goal3.pose.pose.orientation.z = 3.14;
        goal3.max_linear_speed = 0.5; 
        goal3.max_angular_speed = 0.5;
        goals_.push_back(goal3);

        Goal goal4;
        goal4.type = 0;
        goal4.pose.pose.position.x = 0.0;
        goal4.pose.pose.position.y = 0.0;
        goal4.pose.pose.orientation.z = 0.0;
        goal4.max_linear_speed = 2.0; 
        goal4.max_angular_speed = 0.5;
        goals_.push_back(goal4);
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
        if(goals_.front().type == 1) {
            // 如果是攝影機目標，則呼叫攝影機服務
            auto request = std::make_shared<interfaces::srv::Menu::Request>();
            request->start = goals_.front().start;
            auto future = menu_client_->async_send_request(request,std::bind(&MissionTwo::menu_response_callback, this, std::placeholders::_1));
            waiting_for_response_ = true;
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Sent camera goal: start=%s", request->start ? "true" : "false");
        } else {
            // 如果是底盤目標，則呼叫底盤服務
            auto request = std::make_shared<interfaces::srv::GoalPoint::Request>();
            request->goal = goals_.front().pose;
            request->max_linear_speed = goals_.front().max_linear_speed;
            request->max_angular_speed = goals_.front().max_angular_speed;
            auto future = goal_client_->async_send_request(request,std::bind(&MissionTwo::goal_response_callback, this, std::placeholders::_1));
            waiting_for_response_ = true;
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Sent goal: x=%.2f, y=%.2f", request->goal.pose.position.x, request->goal.pose.position.y);
        }
    }

    void goal_response_callback(rclcpp::Client<interfaces::srv::GoalPoint>::SharedFuture future) {
        auto response = future.get();
        if (response->status) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Goal reached.");
            goals_.erase(goals_.begin());
            waiting_for_response_ = false;
        } else {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Goal not reached yet.");
        }
    }

    void menu_response_callback(rclcpp::Client<interfaces::srv::Menu>::SharedFuture future) {
        auto response = future.get();
        if (response->result == -1) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Camera not started.");
        } else {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Camera started.");
            color_id_ = response->result / 10;
            num_ = response->result % 10;
            if(color_id_!=-1 && num_!=-1) {
                RCLCPP_INFO(this->get_logger(), "Color ID: %d, Number: %d", color_id_, num_);
                goals_.erase(goals_.begin());
                waiting_for_response_ = false;
            } else {
                RCLCPP_WARN(this->get_logger(), "Invalid color ID or number.");
            }
        }
        
    }

    struct Goal {
        int type;   //chassis = 0, camera =1
        geometry_msgs::msg::PoseStamped pose;
        double max_linear_speed;
        double max_angular_speed;
        bool start;
    };

    // 成員變數
    rclcpp::Client<interfaces::srv::GoalPoint>::SharedPtr goal_client_;
    rclcpp::Client<interfaces::srv::Menu>::SharedPtr menu_client_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<Goal> goals_;
    bool waiting_for_response_{false};
    int color_id_{-1}, num_{-1};
};