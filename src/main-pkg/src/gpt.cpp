#pragma once
#include <cmath>
#include <algorithm>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "interfaces/srv/goalpoint.hpp"

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class ChassisPilot : public rclcpp::Node {
public:
    ChassisPilot() : Node("chassis_pilot") {
        // Declare parameters (with defaults)
        pos_tol_             = this->declare_parameter<double>("pos_tol", 0.02);   // m
        yaw_tol_             = this->declare_parameter<double>("yaw_tol", 0.03);   // rad
        max_linear_speed_    = this->declare_parameter<double>("max_linear_speed", 0.8);   // m/s
        max_angular_speed_   = this->declare_parameter<double>("max_angular_speed", 1.5);  // rad/s
        linear_acceleration_ = this->declare_parameter<double>("linear_acceleration", 0.8);   // m/s^2
        angular_acceleration_= this->declare_parameter<double>("angular_acceleration", 1.5);  // rad/s^2
        turn_first_          = this->declare_parameter<bool>("turn_first", true);
        angle_for_translation_= this->declare_parameter<double>("angle_for_translation", 0.20); // rad，先把角度對準到這個閾值內再走直線
        log_throttle_ms_     = this->declare_parameter<int>("log_throttle_ms", 500);

        position_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/robot/pose", 10,
            std::bind(&ChassisPilot::position_callback, this, std::placeholders::_1));

        velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/robot/cmd_vel", 10);

        goal_server_ = this->create_service<interfaces::srv::GoalPoint>(
            "/goal",
            std::bind(&ChassisPilot::set_goal, this,
                      std::placeholders::_1, std::placeholders::_2));

        timer_ = this->create_wall_timer(50ms, std::bind(&ChassisPilot::publish_velocity, this));

        RCLCPP_INFO(this->get_logger(), "ChassisPilot started.");
    }

private:
    // ---------------- Callbacks ----------------
    void position_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        x_   = msg->pose.pose.position.x;
        y_   = msg->pose.pose.position.y;
        yaw_ = quat_to_yaw(msg->pose.pose.orientation);

        vx_meas_ = msg->twist.twist.linear.x;
        vy_meas_ = msg->twist.twist.linear.y;
        wz_meas_ = msg->twist.twist.angular.z;

        linear_velocity_now_  = std::hypot(vx_meas_, vy_meas_);
        angular_velocity_now_ = wz_meas_;

        have_state_ = true;
        // 記 log 不要太吵
        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(),
            static_cast<uint64_t>(std::max(2000, log_throttle_ms_)),
            "State x=%.3f y=%.3f yaw=%.3f | v=%.2f w=%.2f",
            x_, y_, yaw_, linear_velocity_now_, angular_velocity_now_);
    }

    void set_goal(const std::shared_ptr<interfaces::srv::GoalPoint::Request> request,
                  const std::shared_ptr<interfaces::srv::GoalPoint::Response> response) {
        goal_x_   = request->pose.position.x;
        goal_y_   = request->pose.position.y;
        goal_yaw_ = quat_to_yaw(request->pose.orientation);

        update_dist(); // refresh dist_to_goal_, yaw_to_goal_

        const bool done = complete_goal_unsafe();
        response->status = done;        // true = 已完成
        have_goal_ = !done;

        RCLCPP_INFO(this->get_logger(),
                    "Goal Set: x=%.3f y=%.3f yaw=%.3f | dist=%.3f d_yaw=%.3f rad | have_goal=%d",
                    goal_x_, goal_y_, goal_yaw_, dist_to_goal_, yaw_to_goal_, (int)have_goal_);
    }

    // ---------------- Control (timer) ----------------
    void publish_velocity() {
        if (!have_state_ || !have_goal_) {
            RCLCPP_INFO_THROTTLE(
                this->get_logger(), *this->get_clock(),
                static_cast<uint64_t>(log_throttle_ms_),
                "[idle] waiting for %s%s",
                have_state_ ? "" : "state ",
                have_goal_  ? "" : "goal");
            return;
        }

        update_dist();

        geometry_msgs::msg::Twist cmd;

        // 已達目標 → 停車一次並清除 have_goal_
        if (complete_goal_unsafe()) {
            reset_twist(cmd);
            velocity_publisher_->publish(cmd);
            have_goal_ = false;
            RCLCPP_INFO(this->get_logger(), "Goal reached. Stopping.");
            return;
        }

        // 控制策略：預設「先轉再走」; 或者在小角度閾值內允許同時平移
        const double dir_world = std::atan2(goal_y_ - y_, goal_x_ - x_);
        const double ang_err   = yaw_to_goal_;          // [-pi, pi]
        const double ang_mag   = std::abs(ang_err);
        const double dist      = dist_to_goal_;

        // 角度控制（trapezoid-like）
        double w_des = std::min(
            max_angular_speed_,
            std::sqrt(std::max(0.0, 2.0 * angular_acceleration_ * ang_mag))
        );
        // 限制加速度（加/減速）
        w_des = clamp_rate(w_des, std::abs(angular_velocity_now_), angular_acceleration_, 0.05);
        w_des *= (ang_err >= 0.0 ? 1.0 : -1.0);  // 加上方向

        // 平移控制（trapezoid-like）
        double v_des = std::min(
            max_linear_speed_,
            std::sqrt(std::max(0.0, 2.0 * linear_acceleration_ * dist))
        );
        v_des = clamp_rate(v_des, linear_velocity_now_, linear_acceleration_, 0.05);

        // 決策：先轉還是可邊走邊轉
        const bool allow_translate = (!turn_first_) || (ang_mag < angle_for_translation_);

        if (allow_translate && !translation_complete_unsafe()) {
            // 世界座標 → 速度向量（對全向底盤 OK；非全向底盤應只用 x 並先對準角度）
            cmd.linear.x  = v_des * std::cos(dir_world);
            cmd.linear.y  = v_des * std::sin(dir_world);
            cmd.angular.z = w_des;
        } else {
            // 只轉向
            cmd.angular.z = w_des;
            cmd.linear.x = cmd.linear.y = 0.0;
        }

        // 安全：若即將到點，避免 overshoot，小角/小距離時進一步限速
        if (dist < 0.15) {
            const double scale = std::clamp(dist / 0.15, 0.0, 1.0);
            cmd.linear.x *= scale;
            cmd.linear.y *= scale;
        }
        if (ang_mag < 0.15) {
            const double scale = std::clamp(ang_mag / 0.15, 0.0, 1.0);
            cmd.angular.z *= scale;
        }

        velocity_publisher_->publish(cmd);
    }

    // ---------------- Helpers ----------------
    bool complete_goal_unsafe() const {
        return translation_complete_unsafe() && rotation_complete_unsafe();
    }
    bool translation_complete_unsafe() const {
        return dist_to_goal_ < pos_tol_;
    }
    bool rotation_complete_unsafe() const {
        return std::abs(yaw_to_goal_) < yaw_tol_;
    }

    void reset_twist(geometry_msgs::msg::Twist &msg) const {
        msg.linear.x = msg.linear.y = msg.linear.z = 0.0;
        msg.angular.x = msg.angular.y = msg.angular.z = 0.0;
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

    void update_dist() {
        dist_to_goal_ = std::hypot(goal_x_ - x_, goal_y_ - y_);
        yaw_to_goal_  = ang_norm(goal_yaw_ - yaw_);
    }

    // 以可接受的 dt（預設 50ms）限制加/減速度，避免跳變
    static double clamp_rate(double target_mag, double current_mag, double a_max, double dt) {
        const double diff = target_mag - current_mag;
        const double lim  = a_max * dt;
        if (diff >  lim) return current_mag + lim;
        if (diff < -lim) return current_mag - lim;
        return target_mag;
    }

    // ---------------- State ----------------
    double x_{0.0}, y_{0.0}, yaw_{0.0};
    double vx_meas_{0.0}, vy_meas_{0.0}, wz_meas_{0.0};
    double linear_velocity_now_{0.0}, angular_velocity_now_{0.0};
    bool have_state_{false};

    // goal
    double goal_x_{0.0}, goal_y_{0.0}, goal_yaw_{0.0};
    bool have_goal_{false};

    // derived errors
    double dist_to_goal_{0.0}, yaw_to_goal_{0.0};

    // params
    double pos_tol_{0.02}, yaw_tol_{0.03};
    double max_linear_speed_{0.8}, max_angular_speed_{1.5};
    double linear_acceleration_{0.8}, angular_acceleration_{1.5};
    bool   turn_first_{true};
    double angle_for_translation_{0.20};
    int    log_throttle_ms_{500};

    // ROS
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr  velocity_publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr position_subscriber_;
    rclcpp::Service<interfaces::srv::GoalPoint>::SharedPtr   goal_server_;
    rclcpp::TimerBase::SharedPtr                             timer_;
};
