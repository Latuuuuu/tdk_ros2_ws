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
        mission_command_sub_ = this->create_subscription<std_msgs::msg::Int32>("/mission_command", 10, std::bind(&MissionTwo::mission_command_callback, this, std::placeholders::_1));
        mission_status_pub_ = this->create_publisher<std_msgs::msg::Int32>("/mission_status", 10);
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
        //int type,double x, double y, double yaw, int move_mode, bool start, int arm_cmd, double max_linear_speed = 20.0, double max_angular_speed = 0.5
        goals_.push_back(create_goal(0, 0.0, 616.0, 4.71, 11, 0, 0, 20.0, 0.5)); //第一關終點
        goals_.push_back(create_goal(0, 0.0, 616.0, 4.71, 21, 0, 0, 20.0, 0.5));
        goals_.push_back(create_goal(0, 0.0, 616.0, 4.71, 10, 0, 0, 20.0, 0.5));

        goals_.push_back(create_goal(0, 83.0, 616.0, 3.14, 11, 0, 0, 20.0, 0.5)); //第二關起點
        goals_.push_back(create_goal(0, 83.0, 616.0, 3.14, 21, 0, 0, 20.0, 0.5));
        goals_.push_back(create_goal(0, 83.0, 616.0, 3.14, 10, 0, 0, 20.0, 0.5));


        goals_.push_back(create_goal(0, 85.0, 666.0, 3.14, 10, 0, 0, 20.0, 0.5)); //去主桌前
        // goals_.push_back(create_goal(0, 85.0, 666.0, 3.14, 21, 0, 0, 20.0, 0.5));

        goals_.push_back(create_goal(3, 85.0, 666.0, 3.14, 11, 0, 1)); //手臂抬鏡頭

        goals_.push_back(create_goal(1, 85.0, 666.0, 3.14, 11, 1, 0)); //菜單辨識

        goals_.push_back(create_goal(3, 85.0, 666.0, 3.14, 11, 0, 2)); //手臂夾杯子


        goals_.push_back(create_goal(0, 83.0, 666.0, 3.14, 10, 0, 0, 20.0, 0.5)); //回到主桌前
        goals_.push_back(create_goal(0, 83.0, 616.0, 1.57, 11, 0, 0, 20.0, 0.5));
        goals_.push_back(create_goal(0, 83.0, 616.0, 1.57, 21, 0, 0, 20.0, 0.5));

        goals_.push_back(create_goal(3, 83.0, 616.0, 1.57, 11, 0, 3)); //手臂放鏡頭

        goals_.push_back(create_goal(2, 83.0, 616.0, 1.57, 11, 1, 0)); //主視覺辨識

        goals_.push_back(create_goal(3, 83.0, 616.0, 1.57, 11, 0, 4)); //手臂放杯子

        goals_.push_back(create_goal(0, -267.0, 666.0, 1.57, 11, 0, 0, 20.0, 0.5)); //第二關終點
        goals_.push_back(create_goal(0, -267.0, 666.0, 1.57, 21, 0, 0, 20.0, 0.5));
    }

    void mission_command_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        if (msg->data == 2 && !node_started_) {
            waiting_for_response_ = false;
            pending_arm_cmd_ = 0;
            waiting_for_arm_ = false;
            node_started_ = true;
        }
        else{
            node_started_ = false;
        }
    }

    void check_and_send_goal() {
        // 如果目前沒有目標，或目標序列已完成，則退出
        if (!node_started_) {
            return;
        }

        if (goals_.empty()) {
            RCLCPP_INFO(this->get_logger(), "Mission Two Completed.");
            // rclcpp::shutdown();
            std_msgs::msg::Int32 status;
            status.data = 2; // Mission Two 的 ID
            mission_status_pub_->publish(status);
            return;
        }

        // 如果正在等待服務回應，則不發送新目標
        if (waiting_for_response_ && goals_.front().type != 3) {
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
            waiting_for_response_ = true;
            auto future = goal_client_->async_send_request(request,std::bind(&MissionTwo::goal_response_callback, this, std::placeholders::_1));
            RCLCPP_INFO(this->get_logger(),"Sent goal: x=%.2f, y=%.2f, move mode:%d", request->goal.pose.position.x, request->goal.pose.position.y, request->move_mode);
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
            goals_.erase(goals_.begin());
            // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Goal reached.");
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
        double x2,x3,y2,y3,z2,z3;
        if (num_ == 1) {
            x2=-151.0; y2=616.0; z2=3.14;
            x3=-148.0; y3=641.0; z3=3.14;
            x_ = -148.0;
            y_ = 641.0;
            z_ = 3.14;
        } else if (num_ == 2) {
            x2=-151.0; y2=616.0; z2=0.0;
            x3=-148.0; y3=591.0; z3=0.0;
            x_ = -148.0;
            y_ = 591.0;
            z_ = 0.0;
        } else if (num_ == 3) {
            x2=-81.0; y2=616.0; z2=0.0;
            x3=-78.0; y3=591.0; z3=0.0;
            x_ = -78.0;
            y_ = 591.0;
            z_ = 0.0;
        } else if (num_ == 4) {
            x2=-81.0; y2=616.0; z2=3.14;
            x3=-78.0; y3=641.0; z3=3.14;
            x_ = -78.0;
            y_ = 641.0;
            z_ = 3.14;
        }
        goals_.insert(goals_.begin()+4,create_goal(0, x2, y2, z2, 11, 0, 0, 20.0, 0.5));
        goals_.insert(goals_.begin()+5,create_goal(0, x2, y2, z2, 21, 0, 0, 20.0, 0.5));
        goals_.insert(goals_.begin()+6,create_goal(0, x3, y3, z3, 10, 0, 0, 20.0, 0.5));
        goals_.insert(goals_.begin()+10,create_goal(0, x3, y3, z3, 10, 0, 0, 20.0, 0.5));
        goals_.insert(goals_.begin()+11,create_goal(0, x3, y3, z3, 11, 0, 0, 20.0, 0.5));
        goals_.insert(goals_.begin()+11,create_goal(0, x2, y2, 1.57, 21, 0, 0, 20.0, 0.5));//去餐桌中點
        goals_.insert(goals_.begin()+12,create_goal(0, x2, y2, 1.57, 21, 0, 0, 20.0, 0.5));//去餐桌中點

        Goal goal;//夾杯子的位置
        goal.type = 0;
        if (color_id_)goals_.insert(goals_.begin(), create_goal(0, 63.0, 666.0, 3.14, 10, 0, 0, 20.0, 0.5));
        else goals_.insert(goals_.begin(), create_goal(0, 103.0, 666.0, 3.14, 10, 0, 0, 20.0, 0.5));
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
        if(y_ > 620)goals_.insert(goals_.begin(), create_goal(0, x_ + dx - 3.0, y_ + (dy / 2.0), z_, 10, 0, 0, 20.0, 0.5));
        else goals_.insert(goals_.begin(), create_goal(0, x_ + dx + 3.0, y_ + (dy / 2.0), z_, 10, 0, 0, 20.0, 0.5));
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

    };

    Goal create_goal(int type,double x, double y, double yaw, int move_mode, bool start, int arm_cmd, double max_linear_speed = 20.0, double max_angular_speed = 0.5) {
        Goal goal;
        goal.type = type;
        goal.pose.pose.position.x = x;
        goal.pose.pose.position.y = y;
        goal.pose.pose.orientation.z = yaw;
        goal.max_linear_speed = max_linear_speed;
        goal.max_angular_speed = max_angular_speed;
        goal.move_mode = move_mode;
        goal.start = start;
        goal.arm_cmd = arm_cmd;
        return goal;
    }

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
    bool node_started_{true};
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr mission_command_sub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr arm_cmd_pub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr arm_status_sub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr mission_status_pub_;
};