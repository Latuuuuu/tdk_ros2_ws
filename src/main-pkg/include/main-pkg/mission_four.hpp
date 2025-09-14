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
        mission_status_pub_ = this->create_publisher<std_msgs::msg::Int32>("/mission_status", 10);
        goal_client_ = this->create_client<interfaces::srv::GoalPoint>("/goal");
        arm_cmd_pub_ = this->create_publisher<std_msgs::msg::Int32>("/robot/cmd_arm", 10);
        arm_status_sub_ = this->create_subscription<std_msgs::msg::Int32>("/robot/arm_status", 10,std::bind(&MissionFour::arm_status_callback, this, std::placeholders::_1));


        // 初始化目標點序列
        initialize_goals();

        // 計時器，用於檢查目標完成狀態並發送下一個目標
        timer_ = this->create_wall_timer(std::chrono::milliseconds(10),std::bind(&MissionFour::check_and_send_goal, this));

        RCLCPP_INFO(this->get_logger(), "MissionFour started.");
    }

private:
    void initialize_goals() {
        // 初始化目標點序列
        Goal goal1;
        goal1.type = 0;
        goal1.pose.pose.position.x = -4920.0;
        goal1.pose.pose.position.y = 3350.0;
        goal1.pose.pose.orientation.z = 3.14;
        goal1.max_linear_speed = 20.0;
        goal1.max_angular_speed = 0.2;
        goals_.push_back(goal1);

        Goal goal2;//第二關起點+轉向背對主桌
        goal2.type = 0;
        goal2.pose.pose.position.x = -4920.0;
        goal2.pose.pose.position.y = 2550.0; //6160.0
        goal2.pose.pose.orientation.z = 3.14;
        goal2.max_linear_speed = 20.0;
        goal2.max_angular_speed = 0.2;
        goals_.push_back(goal2);

        Goal goal3;//走到主桌前
        goal3.type = 0;
        goal3.pose.pose.position.x = -4780.0;
        goal3.pose.pose.position.y = 2550.0;
        goal3.pose.pose.orientation.z = 3.14;
        goal3.max_linear_speed = 20.0;
        goal3.max_angular_speed = 0.2;
        goals_.push_back(goal3);

        Goal goal4;//走到主桌前
        goal4.type = 0;
        goal4.pose.pose.position.x = -4780.0;
        goal4.pose.pose.position.y = 2050.0;
        goal4.pose.pose.orientation.z = 3.14;
        goal4.max_linear_speed = 20.0;
        goal4.max_angular_speed = 0.2;
        goals_.push_back(goal4);

        Goal goal5;//回起點+轉向
        goal5.type = 0;
        goal5.pose.pose.position.x = -3960.0;
        goal5.pose.pose.position.y = 2050.0;
        goal5.pose.pose.orientation.z = 3.14;
        goal5.max_linear_speed = 20.0;
        goal5.max_angular_speed = 0.2;
        goals_.push_back(goal5);

        Goal goal6;//第二關終點
        goal6.type = 0;
        goal6.pose.pose.position.x = -3960.0;
        goal6.pose.pose.position.y = 1820.0;
        goal6.pose.pose.orientation.z = 3.14;
        goal6.max_linear_speed = 20.0;
        goal6.max_angular_speed = 0.2;
        goals_.push_back(goal6);

        Goal goal7;//第二關終點
        goal7.type = 0;
        goal7.pose.pose.position.x = -3150.0;
        goal7.pose.pose.position.y = 1820.0;
        goal7.pose.pose.orientation.z = 3.14;
        goal7.max_linear_speed = 20.0;
        goal7.max_angular_speed = 0.2;
        goals_.push_back(goal7);

        Goal goal8;//第二關終點
        goal8.type = 0;
        goal8.pose.pose.position.x = -3150.0;
        goal8.pose.pose.position.y = 1050.0;
        goal8.pose.pose.orientation.z = 3.14;
        goal8.max_linear_speed = 20.0;
        goal8.max_angular_speed = 0.2;
        goals_.push_back(goal8);

        Goal goal9;//第二關終點
        goal9.type = 0;
        goal9.pose.pose.position.x = -3150.0;
        goal9.pose.pose.position.y = 1050.0;
        goal9.pose.pose.orientation.z = 3.14;
        goal9.max_linear_speed = 20.0;
        goal9.max_angular_speed = 0.2;
        goals_.push_back(goal9);

        Goal goal10;//第二關終點
        goal10.type = 0;
        goal10.pose.pose.position.x = -3960.0;
        goal10.pose.pose.position.y = 1050.0;
        goal10.pose.pose.orientation.z = 3.14;
        goal10.max_linear_speed = 20.0;
        goal10.max_angular_speed = 0.2;
        goals_.push_back(goal10);

        Goal goal11;//第二關終點
        goal11.type = 0;
        goal11.pose.pose.position.x = -3960.0;
        goal11.pose.pose.position.y = 930.0;
        goal11.pose.pose.orientation.z = 3.14;
        goal11.max_linear_speed = 20.0;
        goal11.max_angular_speed = 0.2;
        goals_.push_back(goal11);

        Goal goal12;//第二關終點
        goal12.type = 0;
        goal12.pose.pose.position.x = -6170.0;
        goal12.pose.pose.position.y = 930.0;
        goal12.pose.pose.orientation.z = 3.14;
        goal12.max_linear_speed = 20.0;
        goal12.max_angular_speed = 0.2;
        goals_.push_back(goal12);

        Goal goal13;//第二關終點
        goal13.type = 0;
        goal13.pose.pose.position.x = -6170.0;
        goal13.pose.pose.position.y = 120.0;
        goal13.pose.pose.orientation.z = 3.14;
        goal13.max_linear_speed = 20.0;
        goal13.max_angular_speed = 0.2;
        goals_.push_back(goal13);

        Goal goal14;//第二關終點
        goal14.type = 0;
        goal14.pose.pose.position.x = -4720.0;
        goal14.pose.pose.position.y = 120.0;
        goal14.pose.pose.orientation.z = 3.14;
        goal14.max_linear_speed = 20.0;
        goal14.max_angular_speed = 0.2;
        goals_.push_back(goal14);

        Goal goal15;//第二關終點
        goal15.type = 0;
        goal15.pose.pose.position.x = -4720.0;
        goal15.pose.pose.position.y = 20.0;
        goal15.pose.pose.orientation.z = 3.14;
        goal15.max_linear_speed = 20.0;
        goal15.max_angular_speed = 0.2;
        goals_.push_back(goal15);
    }

    void check_and_send_goal() {
        // 如果目前沒有目標，或目標序列已完成，則退出
        if (goals_.empty()) {
            RCLCPP_INFO(this->get_logger(), "All goals completed.");
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
    };

    // 成員變數
    rclcpp::Client<interfaces::srv::GoalPoint>::SharedPtr goal_client_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<Goal> goals_;
    bool waiting_for_response_{false};
    int pending_arm_cmd_{0};
    bool waiting_for_arm_{false};
    double x_{-81.0}, y_{641.0},z_{3.14}; 
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr arm_cmd_pub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr arm_status_sub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr mission_status_pub_;
};