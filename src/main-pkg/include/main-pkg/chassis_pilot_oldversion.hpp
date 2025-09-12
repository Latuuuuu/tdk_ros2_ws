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
    static inline double signum(double x){ return (x>0) - (x<0); }

    static inline double ramp_towards(double current, double target, double step){
        if (current < target) return std::min(current + step, target);
        else                  return std::max(current - step, target);
    }

    // 依剩餘距離 d 與加速度 a 給出「為了能煞得住」的上限速度：v = sqrt(2 a d)
    static inline double braking_speed(double d, double a, double v_max, double v_min = 0.0){
        if (d <= 0.0 || a <= 0.0) return 0.0;
        double v = std::sqrt(2.0 * a * d);
        v = std::clamp(v, v_min, v_max);
        return v;
    }

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

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(),static_cast<uint64_t>(log_throttle_ms_),"State x=%.3f y=%.3f yaw=%.3f | v=%.2f w=%.2f",x_, y_, yaw_, linear_velocity_now_, angular_velocity_now_);
    }
    
    void set_goal(const std::shared_ptr<interfaces::srv::GoalPoint::Request> request,const std::shared_ptr<interfaces::srv::GoalPoint::Response> response){
        goal_x_ = request->goal.pose.position.x;
        goal_y_ = request->goal.pose.position.y;
        // goal_yaw_ = quat_to_yaw(request->goal.pose.orientation);
        goal_yaw_ = request->goal.pose.orientation.z; 

        max_linear_speed_ = request->max_linear_speed;
        max_angular_speed_ = request->max_angular_speed;

        update_dist();

        const bool done = complete_goal();
        response->status = done;        
        have_goal_ = !done;

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), static_cast<uint64_t>(log_throttle_ms_), "Goal Set: x=%.2f, y=%.2f, yaw=%.2f", goal_x_, goal_y_, goal_yaw_);   
    }

    void publish_velocity() {
        if (!have_state_ || !have_goal_) {
            if (stop_hold_ticks_ > 0) {
            geometry_msgs::msg::Twist zero;
            reset_twist(zero);
            velocity_publisher_->publish(zero);
            --stop_hold_ticks_;
            }
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(),static_cast<uint64_t>(log_throttle_ms_),"[idle] waiting for %s%s",have_state_ ? "" : "state ",have_goal_  ? "" : "goal");
            return;
        }

        update_dist();

        geometry_msgs::msg::Twist msg;

        const double step_lin = std::max(1e-6, linear_acceleration_  * control_dt_);
        const double step_ang = std::max(1e-6, angular_acceleration_ * control_dt_);

        if (complete_goal()) {
            reset_twist(msg);
            velocity_publisher_->publish(msg);
            auto response = std::make_shared<interfaces::srv::GoalPoint::Response>();
            response->status = true;
            have_goal_ = false;
            stop_hold_ticks_ = 2;
            RCLCPP_INFO(this->get_logger(), "Goal reached. Stopping.");
            return;
        }

        // 方向到目標點（僅用於平移分解）
        const double direction = std::atan2(goal_y_ - y_, goal_x_ - x_);

        // ====== 階段 1：先平移，再轉向（延續你原本設計） ======
        if (!translation_complete()) {
            // 煞車上限：用「物理加速度」(m/s^2)
            const double v_min_cruise = (dist_to_goal > dist_buffer_) ? 0.2 : 0.1;
            const double v_target_mag = braking_speed(dist_to_goal,
                                                    /*a=*/linear_acceleration_,
                                                    /*v_max=*/max_linear_speed_,
                                                    /*v_min=*/v_min_cruise);

            // 以步階 step_lin 追目標，不會瞬間跳變
            const double v_new_mag = ramp_towards(linear_velocity_now_, v_target_mag, step_lin);
            msg.linear.x = v_new_mag * std::cos(direction);
            msg.linear.y = v_new_mag * std::sin(direction);
            msg.angular.z = ramp_towards(angular_velocity_now_, 0.0, step_ang);
        } else {
            const double ang_err = yaw_to_goal;  // 已經 ang_norm 過
            const double w_target_mag = braking_speed(std::abs(ang_err),
                                                    /*a=*/angular_acceleration_,
                                                    /*w_max=*/max_angular_speed_);
            const double w_target = w_target_mag * signum(ang_err);
            const double w_new    = ramp_towards(angular_velocity_now_, w_target, step_ang);

            msg.linear.x = ramp_towards(vx_meas_, 0.0, step_lin);
            msg.linear.y = ramp_towards(vy_meas_, 0.0, step_lin);
            msg.angular.z = w_new;
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
    double pos_tol_ {0.004}, yaw_tol_ {0.001};
    int log_throttle_ms_ {20000};
    double dist_to_goal {0.0}, yaw_to_goal {0.0};
    double dist_buffer_ {20.0}, yaw_buffer_ {0.5};
    double max_linear_speed_ {1.25}, max_angular_speed_ {1.5}, linear_acceleration_ {0.5}, angular_acceleration_ {0.05};
    double linear_velocity_now_ {0.0}, angular_velocity_now_ {0.0};
    double control_dt_ {0.05};      
    int    stop_hold_ticks_ {0}; 

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr     velocity_publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr    position_subscriber_;
    rclcpp::Service<interfaces::srv::GoalPoint>::SharedPtr      goal_server_;
      rclcpp::TimerBase::SharedPtr  timer_;

};

