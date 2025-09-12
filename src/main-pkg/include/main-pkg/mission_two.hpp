#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "interfaces/srv/goal_point.hpp"
#include "interfaces/srv/menu.hpp"
#include "interfaces/srv/key_visual.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/int32.hpp"

class MissionTwo : public rclcpp::Node {
public:
    MissionTwo() : Node("mission_2") {
        // 建立目標點的 client
        goal_client_ = this->create_client<interfaces::srv::GoalPoint>("/goal");
        menu_client_ = this->create_client<interfaces::srv::Menu>("/menu");
        table_client_ = this->create_client<interfaces::srv::KeyVisual>("/table");
        arm_cmd_pub_ = this->create_publisher<std_msgs::msg::Int32>("/robot/cmd_arm", 10);
        arm_status_sub_ = this->create_subscription<std_msgs::msg::Int32>("/robot/arm_status", 10,std::bind(&MissionTwo::arm_status_callback, this, std::placeholders::_1));

        // 初始化目標點序列
        initialize_goals();

        // 計時器，用於檢查目標完成狀態並發送下一個目標
        timer_ = this->create_wall_timer(std::chrono::milliseconds(10),std::bind(&MissionTwo::check_and_send_goal, this));

        RCLCPP_INFO(this->get_logger(), "MissionTwo started.");
    }

private:
    void initialize_goals() {
        // 初始化目標點序列
        Goal goal1;// 第一關至第二關的銜接點
        goal1.type = 0;
        goal1.pose.pose.position.x = 0.0;
        goal1.pose.pose.position.y = 4500.0; //6160.0
        goal1.pose.pose.orientation.z = 0.0;
        goal1.max_linear_speed = 20.0;
        goal1.max_angular_speed = 0.01;
        goals_.push_back(goal1);

        Goal goal2;//第二關起點+轉向背對主桌
        goal2.type = 0;
        goal2.pose.pose.position.x = 830.0;
        goal2.pose.pose.position.y = 4500.0; //6160.0
        goal2.pose.pose.orientation.z = 3.14;
        goal2.max_linear_speed = 20.0;
        goal2.max_angular_speed = 0.2;
        goals_.push_back(goal2);

        // Goal goal4;//走到主桌前
        // goal4.type = 0;
        // goal4.pose.pose.position.x = 850.0;
        // goal4.pose.pose.position.y = 6660.0;
        // goal4.pose.pose.orientation.z = 3.14;
        // goal4.max_linear_speed = 20.0;
        // goal4.max_angular_speed = 0.2;
        // goals_.push_back(goal4);

        Goal goal5;
        goal5.type = 3;
        goal5.arm_cmd = 1;
        goals_.push_back(goal5);

        Goal goal6;
        goal6.type = 1;
        goal6.start = 1;
        goals_.push_back(goal6);

        Goal goal7;
        goal7.type = 3;
        goal7.arm_cmd = 2;
        goals_.push_back(goal7);

        Goal goal8;//走到主桌前
        goal8.type = 0;
        goal8.pose.pose.position.x = 830.0;
        goal8.pose.pose.position.y = 6660.0;
        goal8.pose.pose.orientation.z = 3.14;
        goal8.max_linear_speed = 20.0;
        goal8.max_angular_speed = 0.2;
        goals_.push_back(goal8);

        Goal goal9;//回起點+轉向
        goal9.type = 0;
        goal9.pose.pose.position.x = 830.0;
        goal9.pose.pose.position.y = 6160.0;
        goal9.pose.pose.orientation.z = 1.57;
        goal9.max_linear_speed = 20.0;
        goal9.max_angular_speed = 0.2;
        goals_.push_back(goal9);

        Goal goal10;
        goal10.type = 3;
        goal10.arm_cmd = 3;
        goals_.push_back(goal10);

        Goal goal11;
        goal11.type = 2;
        goal11.start = 1;
        goals_.push_back(goal11);

        Goal goal12;
        goal12.type = 3;
        goal12.arm_cmd = 4;
        goals_.push_back(goal12);

        Goal goal13;//第二關終點
        goal13.type = 0;
        goal13.pose.pose.position.x = -2670.0;
        goal13.pose.pose.position.y = 6660.0;
        goal13.pose.pose.orientation.z = 1.57;
        goal13.max_linear_speed = 20.0;
        goal13.max_angular_speed = 0.2;
        goals_.push_back(goal13);
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
            if (!menu_client_->wait_for_service(std::chrono::seconds(1))) {
                RCLCPP_WARN(this->get_logger(), "Menu service not available yet...");
                return; 
            }
            auto request = std::make_shared<interfaces::srv::Menu::Request>();
            request->start = goals_.front().start;
            auto future = menu_client_->async_send_request(request,std::bind(&MissionTwo::menu_response_callback, this, std::placeholders::_1));
            waiting_for_response_ = true;
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Sent camera goal: start=%s", request->start ? "true" : "false");
        } else if(goals_.front().type == 0){
            // 如果是底盤目標，則呼叫底盤服務
            if (!goal_client_->wait_for_service(std::chrono::seconds(1))) {
                RCLCPP_WARN(this->get_logger(), "GoalPoint service not available yet...");
                return;
            }
            auto request = std::make_shared<interfaces::srv::GoalPoint::Request>();
            request->goal = goals_.front().pose;
            request->max_linear_speed = goals_.front().max_linear_speed;
            request->max_angular_speed = goals_.front().max_angular_speed;
            auto future = goal_client_->async_send_request(request,std::bind(&MissionTwo::goal_response_callback, this, std::placeholders::_1));
            waiting_for_response_ = true;
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 20000, "Sent goal: x=%.2f, y=%.2f", request->goal.pose.position.x, request->goal.pose.position.y);
        }else if (goals_.front().type == 3) {
            // 手臂任務：發布行動代號並等待回覆等於同代號
            pending_arm_cmd_ = goals_.front().arm_cmd;
            std_msgs::msg::Int32 cmd;
            cmd.data = pending_arm_cmd_;
            arm_cmd_pub_->publish(cmd);
            waiting_for_response_ = true;
            waiting_for_arm_ = true;
            RCLCPP_INFO(this->get_logger(), "Sent arm cmd: %d", pending_arm_cmd_);
        }else{
            // 如果是桌子目標，則呼叫桌子服務
            if (!table_client_->wait_for_service(std::chrono::seconds(1))) {
                RCLCPP_WARN(this->get_logger(), "Table service not available yet...");
                return;
            }
            auto request = std::make_shared<interfaces::srv::KeyVisual::Request>();
            request->start = goals_.front().start;
            auto future = table_client_->async_send_request(request,std::bind(&MissionTwo::table_response_callback, this, std::placeholders::_1));
            waiting_for_response_ = true;
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Sent table goal: start=%s", request->start ? "true" : "false");
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

    void menu_response_callback(rclcpp::Client<interfaces::srv::Menu>::SharedFuture future) {
        auto response = future.get();
        color_id_ = 0;
        num_ = 4;
        if (response->result == -1) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Camera not started.");
        } else {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Camera started.");
            color_id_ = response->result / 10;
            num_ = response->result % 10;
            if(color_id_!=-1 && num_!=-1) {
                RCLCPP_INFO(this->get_logger(), "Color ID: %d, Number: %d", color_id_, num_);//black0 white1  
            } else {
                RCLCPP_WARN(this->get_logger(), "Invalid color ID or number.");
            }
        }
        waiting_for_response_ = false;
        goals_.erase(goals_.begin());
        Goal goal2;//去正確的餐桌中點
            Goal goal3;//去正確的餐桌前
            goal2.type = 0;
            goal3.type = 0;
            if (num_ == 1) {
                goal2.pose.pose.position.x = -1510.0;
                goal2.pose.pose.position.y = 6160.0;
                goal2.pose.pose.orientation.z = 3.14;
                goal3.pose.pose.position.x = -1480.0;
                goal3.pose.pose.position.y = 6410.0;
                goal3.pose.pose.orientation.z = 3.14;
                x_ = -1480.0;
                y_ = 6410.0;
                z_ = 3.14;
            } else if (num_ == 2) {
                goal2.pose.pose.position.x = -1510.0;
                goal2.pose.pose.position.y = 6160.0;
                goal2.pose.pose.orientation.z = 0.0;
                goal3.pose.pose.position.x = -1480.0;
                goal3.pose.pose.position.y = 5910.0;
                goal3.pose.pose.orientation.z = 0.0;
                x_ = -1480.0;
                y_ = 5910.0;
                z_ = 0.0;
            } else if (num_ == 3) {
                goal2.pose.pose.position.x = -810.0;
                goal2.pose.pose.position.y = 6160.0;
                goal2.pose.pose.orientation.z = 0.0;
                goal3.pose.pose.position.x = -780.0;
                goal3.pose.pose.position.y = 5910.0;
                goal3.pose.pose.orientation.z = 0.0;
                x_ = -780.0;
                y_ = 5910.0;
                z_ = 0.0;
            } else if (num_ == 4) {
                goal2.pose.pose.position.x = -810.0;
                goal2.pose.pose.position.y = 6160.0;
                goal2.pose.pose.orientation.z = 3.14;
                goal3.pose.pose.position.x = -780.0;
                goal3.pose.pose.position.y = 6410.0;
                goal3.pose.pose.orientation.z = 3.14;
                x_ = -780.0;
                y_ = 6410.0;
                z_ = 3.14;
            }
            goal2.max_linear_speed = 20.0;
            goal2.max_angular_speed = 0.2;
            goal3.max_linear_speed = 20.0;
            goal3.max_angular_speed = 0.01;
            goals_.insert(goals_.begin()+6, goal3);
            goals_.insert(goals_.begin()+7, goal2);
            goals_.insert(goals_.begin()+3, goal2);
            goals_.insert(goals_.begin()+4, goal3);
            
            Goal goal;//夾杯子的位置
            goal.type = 0;
            if (color_id_) goal.pose.pose.position.x = 630.0; else goal.pose.pose.position.x = 1030.0;
            goal.pose.pose.position.y = 6660.0;
            goal.pose.pose.orientation.z = 3.14;
            // goal.max_linear_speed = 20.0;
            goal.max_angular_speed = 0.2;
            goals_.insert(goals_.begin(), goal);
    }

    void table_response_callback(rclcpp::Client<interfaces::srv::KeyVisual>::SharedFuture future) {
        auto response = future.get();
        double dx=0.0, dy=0.0;
        if (response->ok) {
            dx = response->dx;
            dy = response->dy;
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Table detected at (%.1f, %.1f) with %d inliers.", response->cx, response->cy, response->inliers);
            RCLCPP_INFO(this->get_logger(), "Estimated offsets: dx=%.2f, dy=%.2f", dx, dy);
        } else {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Table not detected.");
        }
        waiting_for_response_ = false;
        goals_.erase(goals_.begin());
        Goal goal;//夾取點
        goal.type = 0;
        if(y_ > 6200)goal.pose.pose.position.x = x_ + dx - 30.0;
        else goal.pose.pose.position.x = x_ + dx + 30.0;
        goal.pose.pose.position.y = y_ + ( dy / 2.0 );
        goal.pose.pose.orientation.z = z_;
        goal.max_linear_speed = 20.0;
        goal.max_angular_speed = 0.01;
        goals_.insert(goals_.begin(), goal);
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
    rclcpp::Client<interfaces::srv::Menu>::SharedPtr menu_client_;
    rclcpp::Client<interfaces::srv::KeyVisual>::SharedPtr table_client_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<Goal> goals_;
    bool waiting_for_response_{false};
    int color_id_{-1}, num_{-1};
    double x_{-81.0}, y_{641.0},z_{3.14}; 
    int pending_arm_cmd_{0};
    bool waiting_for_arm_{false};
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr arm_cmd_pub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr arm_status_sub_;
};