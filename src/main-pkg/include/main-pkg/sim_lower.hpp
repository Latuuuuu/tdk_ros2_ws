#pragma once
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <cmath>
#include <string>

class SimLower : public rclcpp::Node {
public:
  SimLower() : rclcpp::Node("sim_lower") {
    // 訂閱速度指令
    sub_cmd_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_topic_, 10, [this](geometry_msgs::msg::Twist::SharedPtr msg) {
        vx_ = msg->linear.x;
        vy_ = msg->linear.y;
        wz_ = msg->angular.z;
      });

    // 發佈里程計
    pub_odom_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);

    // 計時器
    timer_ = create_wall_timer(
      std::chrono::milliseconds(period_ms_),
      std::bind(&SimLower::on_timer_, this));

    last_step_time_ = now();
    RCLCPP_INFO(this->get_logger(), "SimLower started.");
  }

private:
  void on_timer_() {
    // 計算時間差
    const auto current_time = now();
    const double dt = (current_time - last_step_time_).seconds();
    last_step_time_ = current_time;

    // 積分計算位移
    // const double c = std::cos(yaw_), s = std::sin(yaw_);
    x_ += vx_ * dt;
    y_ += vy_ * dt;
    yaw_ += wz_ * dt;
    yaw_ = ang_norm(yaw_);

    // 發佈里程計
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = frame_id_;
    odom.child_frame_id = child_frame_id_;
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.orientation.z = yaw_;
    odom.twist.twist.linear.x = vx_;
    odom.twist.twist.linear.y = vy_;
    odom.twist.twist.angular.z = wz_;
    pub_odom_->publish(odom);
  }

  static double ang_norm(double a) {
        while (a >  M_PI) a -= 2.0 * M_PI;
        while (a < -M_PI) a += 2.0 * M_PI;
        return a;
    }

  static geometry_msgs::msg::Quaternion yaw_to_quaternion(double yaw) {
    geometry_msgs::msg::Quaternion q;
    q.x = 0.0;
    q.y = 0.0;
    q.z = std::sin(yaw * 0.5);
    q.w = std::cos(yaw * 0.5);
    return q;
  }

  // 參數
  const int period_ms_{50};
  const std::string cmd_topic_{"/robot/cmd_vel"};
  const std::string odom_topic_{"/robot/pose"};
  const std::string frame_id_{"odom"};
  const std::string child_frame_id_{"base_link"};

  // 狀態
  double x_{0.0}, y_{0.0}, yaw_{0.0};
  double vx_{0.0}, vy_{0.0}, wz_{0.0};
  rclcpp::Time last_step_time_;

  // ROS 介面
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_;
  rclcpp::TimerBase::SharedPtr timer_;
};