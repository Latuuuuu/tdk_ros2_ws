#pragma once
#include <cmath>  
#include <algorithm>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/odometry.hpp" 
#include "interfaces/srv/goal_point.hpp"

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class ChassisPilot : public rclcpp::Node {
public:
    ChassisPilot() : Node("chassis_pilot") {
        position_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>("/robot/pose", 10, std::bind(&ChassisPilot::position_callback, this, std::placeholders::_1));
        velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/robot/cmd_vel", 10);    
        goal_server_ = this->create_service<interfaces::srv::GoalPoint>("/goal", std::bind(&ChassisPilot::set_goal, this, std::placeholders::_1, std::placeholders::_2));

        timer_ = this->create_wall_timer(std::chrono::milliseconds(50),std::bind(&ChassisPilot::publish_velocity, this));
        
        RCLCPP_INFO(this->get_logger(), "ChassisPilot started.");
    }

private:


    void position_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        x_ = msg->pose.pose.position.x;
        y_ = msg->pose.pose.position.y;
        // yaw_ = quat_to_yaw(msg->pose.pose.orientation);
        yaw_ = msg->pose.pose.orientation.z; 

        vx_meas_ = msg->twist.twist.linear.x;
        vy_meas_ = msg->twist.twist.linear.y;
        wz_meas_ = msg->twist.twist.angular.z;

        linear_velocity_now_ = std::hypot(vx_meas_, vy_meas_);
        angular_velocity_now_ = wz_meas_;

        have_state_ = true;

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(),static_cast<uint64_t>(std::max(2000, log_throttle_ms_)),"State x=%.3f y=%.3f yaw=%.3f | v=%.2f w=%.2f",x_, y_, yaw_, linear_velocity_now_, angular_velocity_now_);
    }
    void set_goal(const std::shared_ptr<interfaces::srv::GoalPoint::Request> request,const std::shared_ptr<interfaces::srv::GoalPoint::Response> response){
        goal_x_ = request->goal.pose.position.x;
        goal_y_ = request->goal.pose.position.y;
        // goal_yaw_ = quat_to_yaw(request->goal.pose.orientation);
        goal_yaw_ = request->goal.pose.orientation.z; 

        update_dist();

        const bool done = complete_goal();
        response->status = done;        
        have_goal_ = !done;

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Goal Set: x=%.2f, y=%.2f, yaw=%.2f", goal_x_, goal_y_, goal_yaw_);   
    }

    void publish_velocity() {
        if (!have_state_ || !have_goal_) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(),static_cast<uint64_t>(log_throttle_ms_),"[idle] waiting for %s%s",have_state_ ? "" : "state ",have_goal_  ? "" : "goal");
            return;
        }

        update_dist();

        geometry_msgs::msg::Twist msg;
        
        if(complete_goal()) {
            reset_twist(msg);
            velocity_publisher_->publish(msg);
            auto response = std::make_shared<interfaces::srv::GoalPoint::Response>();
            response->status = true;
            have_goal_ = false;
            RCLCPP_INFO(this->get_logger(), "Goal reached. Stopping.");
            return; // 已到達目標
        }

        double direction = std::atan2(goal_y_ - y_, goal_x_ - x_);

        if (translation_complete()) {// Angular movement
            double angular_velocity = 0.0;
            if (yaw_to_goal>=yaw_buffer_){
                if(angular_velocity_now_ >= max_angular_speed_) { 
                    angular_velocity = std::min(max_angular_speed_, angular_velocity_now_);
                } else {
                    angular_velocity = angular_velocity_now_ + angular_acceleration_;
                }
            } else if (angular_velocity_now_ / yaw_to_goal <= max_angular_speed_ / yaw_buffer_) {
                    angular_velocity = angular_velocity_now_ + angular_acceleration_;
            } else {
                angular_velocity = std::max(0.0, angular_velocity_now_ - angular_acceleration_);
            }
            msg.linear.x = 0.0;
            msg.linear.y = 0.0;
            msg.angular.z = std::min(1.0, std::abs(angular_velocity)) * std::copysign(1.0, yaw_to_goal); // Normalize to [-1, 1] and apply sign
        } 
        else {// Linear movement
            double linear_velocity = 0.0; 
            if (dist_to_goal>=dist_buffer_){
                if(linear_velocity_now_ >= max_linear_speed_) { 
                    linear_velocity = std::min(max_linear_speed_, linear_velocity_now_);
                } else {
                    linear_velocity = linear_velocity_now_ + linear_acceleration_;
                }
            } else if (linear_velocity_now_ / dist_to_goal <= max_linear_speed_ / dist_buffer_) {
                    linear_velocity = linear_velocity_now_ + linear_acceleration_;
            } else {
                linear_velocity = std::max(0.0, linear_velocity_now_ - linear_acceleration_);
            }
            msg.linear.x = linear_velocity * std::cos(direction);
            msg.linear.y = linear_velocity * std::sin(direction);
            msg.angular.z = 0.0; 
        }

        velocity_publisher_->publish(msg);
    }

    bool complete_goal() {
        return (translation_complete() && rotation_complete());
    }

    bool translation_complete() {
        return dist_to_goal < pos_tol_;
    }

    bool rotation_complete() {
        return std::abs(yaw_to_goal) < yaw_tol_;
    }

    void reset_twist(geometry_msgs::msg::Twist &msg) {
        msg.linear.x = 0;
        msg.linear.y = 0;
        msg.linear.z = 0;
        msg.angular.x = 0;
        msg.angular.y = 0;
        msg.angular.z = 0;
    }

    static double quat_to_yaw(const geometry_msgs::msg::Quaternion &q) {
        const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        return std::atan2(siny_cosp, cosy_cosp);
    }

    static double ang_norm(double a) {
        while (a >  M_PI) a -= 2.0 * M_PI;
        while (a < -M_PI) a += 2.0 * M_PI;
        return a;
    }

    void update_dist(){
        dist_to_goal = std::hypot(goal_x_ - x_, goal_y_ - y_);
        yaw_to_goal = ang_norm(goal_yaw_ - yaw_);
    }

    //status
    double x_ {0.0},y_ {0.0},yaw_ {0.0};
    double vx_meas_ {0.0},vy_meas_ {0.0},wz_meas_ {0.0};
    bool have_state_ {false};
    //goal
    double goal_x_ {0.0},goal_y_ {0.0},goal_yaw_ {0.0};
    bool have_goal_ {false};
    //parameter
    double pos_tol_ {0.02}, yaw_tol_ {0.03};
    int log_throttle_ms_ {200};
    double dist_to_goal {0.0}, yaw_to_goal {0.0};
    double dist_buffer_ {0.0}, yaw_buffer_ {0.0};
    double max_linear_speed_ {1.0}, max_angular_speed_ {1.5}, linear_acceleration_ {0.5}, angular_acceleration_ {0.5};
    double linear_velocity_now_ {0.0}, angular_velocity_now_ {0.0};

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr     velocity_publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr    position_subscriber_;
    rclcpp::Service<interfaces::srv::GoalPoint>::SharedPtr      goal_server_;
      rclcpp::TimerBase::SharedPtr  timer_;

};

