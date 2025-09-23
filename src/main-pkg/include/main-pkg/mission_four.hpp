#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "interfaces/srv/goal_point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/int32.hpp"

class MissionFour : public rclcpp::Node {
public:
    MissionFour() : Node("mission_4") {
        // 建立目標點的 client
        mission_command_sub_ = this->create_subscription<std_msgs::msg::Int32>("/mission_command", 10, std::bind(&MissionFour::mission_command_callback, this, std::placeholders::_1));
        mission_status_pub_ = this->create_publisher<std_msgs::msg::Int32>("/mission_status", 10);
        goal_client_ = this->create_client<interfaces::srv::GoalPoint>("/goal");
        arm_cmd_pub_ = this->create_publisher<std_msgs::msg::Int32>("/robot/cmd_arm", 10);
        arm_status_sub_ = this->create_subscription<std_msgs::msg::Int32>("/robot/arm_status", 10,std::bind(&MissionFour::arm_status_callback, this, std::placeholders::_1));


        // 初始化目標點序列
        initialize_goals();

        // 計時器，用於檢查目標完成狀態並發送下一個目標
        timer_ = this->create_wall_timer(std::chrono::milliseconds(50),std::bind(&MissionFour::check_and_send_goal, this));

        RCLCPP_INFO(this->get_logger(), "MissionFour started.");
    }

private:
    void initialize_goals() {
        // 初始化目標點序列
        goals_.push_back(create_goal(0, -500.0, 500.0, 0.0, 10, 0, 100.0, 0.5)); 

        // goals_.push_back(create_goal(0, -492.0, 335.0, 3.14, 10, 0, 20.0, 0.5, 6)); 

        // goals_.push_back(create_goal(0, -487.0, 260.0, 3.14, 10, 0, 20.0, 0.5));

        // goals_.push_back(create_goal(0, -462.0, 205.0, 3.14, 10, 0, 20.0, 0.5));

        // goals_.push_back(create_goal(0, -262.0, 185.0, 3.14, 10, 0, 20.0, 0.5));

        // goals_.push_back(create_goal(0, -262.0, 115.0, 3.14, 10, 0, 20.0, 0.5));

        // goals_.push_back(create_goal(0, -602.0, 90.0, 3.14, 10, 0, 20.0, 0.5));

        // goals_.push_back(create_goal(0, -602.0, 10.0, 3.14, 10, 0, 20.0, 0.5));

        // goals_.push_back(create_goal(0, -447.0, 0.0, 3.14, 10, 0, 20.0, 0.5));

    }

    void mission_command_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        if (msg->data == 4 && !node_started_) { // 假設 4 是啟動 Mission Four 的命令
            // RCLCPP_INFO(this->get_logger(), "Mission Four command received.");
            // 重新初始化目標點序列
            // goals_.clear();
            // initialize_goals();
            waiting_for_response_ = false;
            pending_arm_cmd_ = 0;
            waiting_for_arm_ = false;
            node_started_ = true;
        }
        if(msg->data == -1 && node_started_){
            node_started_ = false;
            goals_.clear();
            initialize_goals();
        }
    }

    void check_and_send_goal() {
        // 如果目前沒有目標，或目標序列已完成，則退出
        if (!node_started_) {
            return;
        }

        if (goals_.empty()) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 20000, "All goals completed.");
            // rclcpp::shutdown();
            std_msgs::msg::Int32 status;
            status.data = 4; // Mission Four 的 ID
            mission_status_pub_->publish(status);
            return;
        }

        // 如果正在等待服務回應，則不發送新目標
        if (waiting_for_response_) {
            return;
        }

        // 發送下一個目標
        if(goals_.front().type == 0) {
            // 如果是底盤目標，則呼叫底盤服務
            if (!goal_client_->wait_for_service(std::chrono::seconds(1))) {
                RCLCPP_WARN(this->get_logger(), "GoalPoint service not available yet...");
                return;
            }
            auto request = std::make_shared<interfaces::srv::GoalPoint::Request>();
            request->goal = goals_.front().pose;
            request->max_linear_speed = goals_.front().max_linear_speed;
            request->max_angular_speed = goals_.front().max_angular_speed;
            request->move_mode = goals_.front().move_mode;
            auto future = goal_client_->async_send_request(request,std::bind(&MissionFour::goal_response_callback, this, std::placeholders::_1));
            waiting_for_response_ = true;
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 20000, "Sent goal: x=%.2f, y=%.2f", request->goal.pose.position.x, request->goal.pose.position.y);
        }
        else if (goals_.front().type == 3) {
            // 手臂任務：發布行動代號並等待回覆等於同代號
            pending_arm_cmd_ = goals_.front().arm_cmd;
            std_msgs::msg::Int32 cmd;
            cmd.data = pending_arm_cmd_;
            arm_cmd_pub_->publish(cmd);
            waiting_for_response_ = true;
            waiting_for_arm_ = true;
            RCLCPP_INFO(this->get_logger(), "Sent arm cmd: %d", pending_arm_cmd_);
        }
    }

    void goal_response_callback(rclcpp::Client<interfaces::srv::GoalPoint>::SharedFuture future) {
        auto response = future.get();
        if (response->status) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Goal reached.");
            goals_.erase(goals_.begin());
            
        } else {
            // RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Goal not reached yet.");
        }
        waiting_for_response_ = false;
    }

    void arm_status_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        if (!waiting_for_arm_) return;
        if (msg->data == pending_arm_cmd_) {
            RCLCPP_INFO(this->get_logger(), "Arm done for cmd=%d", pending_arm_cmd_);
            waiting_for_response_ = false;
            waiting_for_arm_ = false;
            pending_arm_cmd_ = -1;
            // 完成後彈出此手臂目標，進下一個
            if (!goals_.empty() && goals_.front().type == 3) {
                goals_.erase(goals_.begin());
            }
        } else {
            // 其他代碼：忽略或打印
            RCLCPP_DEBUG(this->get_logger(), "Arm status=%d (waiting %d)", msg->data, pending_arm_cmd_);
        }
    }

    struct Goal {
        int type;   //chassis = 0, camera_menu =1, camera_table =2, arm =3
        geometry_msgs::msg::PoseStamped pose;
        double max_linear_speed;
        double max_angular_speed;
        bool start;
        int arm_cmd ;
        int move_mode; //10: translation, 20: rotation clockwise, 30: rotation counterclockwise, 01: trace, 00: not trace
        int inter_code;
    };

    Goal create_goal(int type,double x, double y, double yaw, int move_mode, int arm_cmd, double max_linear_speed = 20.0, double max_angular_speed = 0.5, int inter_code = 0) {
        Goal goal;
        goal.type = type;
        goal.pose.pose.position.x = x;
        goal.pose.pose.position.y = y;
        goal.pose.pose.orientation.z = yaw;
        goal.max_linear_speed = max_linear_speed;
        goal.max_angular_speed = max_angular_speed;
        goal.move_mode = move_mode;
        goal.arm_cmd = arm_cmd;
        goal.inter_code = inter_code;
        return goal;
    }

    // 成員變數
    rclcpp::Client<interfaces::srv::GoalPoint>::SharedPtr goal_client_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<Goal> goals_;
    bool waiting_for_response_{false};
    int pending_arm_cmd_{0};
    bool waiting_for_arm_{false};
    bool node_started_{false};
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr mission_command_sub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr arm_cmd_pub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr arm_status_sub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr mission_status_pub_;
};