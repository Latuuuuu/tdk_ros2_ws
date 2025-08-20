#pragma once
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <cmath>
#include <algorithm>
#include <string>

class SimLower : public rclcpp::Node {
public:
  SimLower() : rclcpp::Node("sim_lower")
  {
    // ===== 參數 =====
    period_ms_        = declare_parameter<int>("period_ms", 20);          // 模擬步長(ms)
    cmd_topic_        = declare_parameter<std::string>("cmd_topic", "/robot/cmd_vel");
    odom_topic_       = declare_parameter<std::string>("odom_topic", "/robot/pose");
    allow_holonomic_  = declare_parameter<bool>("allow_holonomic", true);

    // 速度/加速度上限（模擬本體硬體能力）
    v_lin_max_ = declare_parameter<double>("v_lin_max", 1.2);   // m/s
    v_ang_max_ = declare_parameter<double>("v_ang_max", 2.5);   // rad/s
    a_lin_max_ = declare_parameter<double>("a_lin_max", 3.0);   // m/s^2
    a_ang_max_ = declare_parameter<double>("a_ang_max", 6.0);   // rad/s^2

    // 一階系統時滯（越小越貼近指令；=0 代表僅受加速度上限）
    tau_lin_   = declare_parameter<double>("tau_lin", 0.15);    // s
    tau_ang_   = declare_parameter<double>("tau_ang", 0.10);    // s

    // 其他
    cmd_timeout_ms_  = declare_parameter<int>("cmd_timeout_ms", 500); // 超時自動煞停
    log_throttle_ms_ = declare_parameter<int>("log_throttle_ms", 200);
    frame_id_        = declare_parameter<std::string>("frame_id", "odom");
    child_frame_id_  = declare_parameter<std::string>("child_frame_id", "base_link");

    // ===== 介面 =====
    pub_odom_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 20);
    sub_cmd_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_topic_, 50, [this](geometry_msgs::msg::Twist::SharedPtr msg){
        cmd_vx_ = std::clamp(msg->linear.x,  -v_lin_max_, v_lin_max_);
        cmd_vy_ = allow_holonomic_
                  ? std::clamp(msg->linear.y,  -v_lin_max_, v_lin_max_) : 0.0;
        cmd_wz_ = std::clamp(msg->angular.z, -v_ang_max_, v_ang_max_);
        last_cmd_time_ = now();
      });

    // 計時器
    set_timer_();

    RCLCPP_INFO(this->get_logger(), "Clock type: %d", this->get_clock()->get_clock_type());

    last_step_time_ = now();
  }

private:
  void set_timer_() {
    using namespace std::chrono_literals;
    timer_ = create_wall_timer(
      std::chrono::milliseconds(period_ms_),
      std::bind(&SimLower::on_timer_, this));
  }

  void on_timer_() {
    const auto t = now();
    double dt = (t - last_step_time_).seconds();
    if (dt <= 0.0) dt = static_cast<double>(period_ms_) / 1000.0;
    last_step_time_ = t;

    // 指令超時 → 漸停
    const bool timeout = (t - last_cmd_time_).nanoseconds() >
                         static_cast<int64_t>(cmd_timeout_ms_) * 1000000LL;
    double tgt_vx = timeout ? 0.0 : cmd_vx_;
    double tgt_vy = timeout ? 0.0 : cmd_vy_;
    double tgt_wz = timeout ? 0.0 : cmd_wz_;

    // 一階滯後 + 加速度限制（vx, vy, wz）
    vx_ = first_order_with_acc_limit_(vx_, tgt_vx, tau_lin_, a_lin_max_, dt);
    vy_ = first_order_with_acc_limit_(vy_, tgt_vy, tau_lin_, a_lin_max_, dt);
    wz_ = first_order_with_acc_limit_(wz_, tgt_wz, tau_ang_, a_ang_max_, dt);

    // 飽和（硬體極限）
    vx_ = std::clamp(vx_, -v_lin_max_, v_lin_max_);
    vy_ = std::clamp(vy_, -v_lin_max_, v_lin_max_);
    wz_ = std::clamp(wz_, -v_ang_max_, v_ang_max_);

    // 本體座標 → 世界座標
    const double c = std::cos(yaw_), s = std::sin(yaw_);
    const double vx_w =  c * vx_ - s * vy_;
    const double vy_w =  s * vx_ + c * vy_;

    // 積分
    x_   += vx_w * dt;
    y_   += vy_w * dt;
    yaw_ += wz_  * dt;
    yaw_ = norm_angle_(yaw_);

    // 發佈 odom
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = t;
    odom.header.frame_id = frame_id_;
    odom.child_frame_id  = child_frame_id_;
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    yaw_to_quat_(yaw_, odom.pose.pose.orientation);
    odom.twist.twist.linear.x  = vx_;
    odom.twist.twist.linear.y  = vy_;
    odom.twist.twist.angular.z = wz_;
    pub_odom_->publish(odom);

    // 診斷輸出
    if (log_throttle_ms_ > 0) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(),
        static_cast<uint64_t>(log_throttle_ms_),
        "[sim] x=%.3f y=%.3f yaw=%.3f | vx=%.3f vy=%.3f wz=%.3f (tgt %.3f %.3f %.3f)%s",
        x_, y_, yaw_, vx_, vy_, wz_, tgt_vx, tgt_vy, tgt_wz, timeout ? " TIMEOUT":"");
    }
  }

  static double norm_angle_(double a) {
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
  }

  static void yaw_to_quat_(double yaw, geometry_msgs::msg::Quaternion &q) {
    q.x = 0.0; q.y = 0.0;
    q.z = std::sin(yaw * 0.5);
    q.w = std::cos(yaw * 0.5);
  }

  // 一階響應 + 加速度限制
  static double first_order_with_acc_limit_(double cur, double tgt, double tau,
                                            double a_max, double dt) {
    double dv_des = (tau > 1e-6) ? (tgt - cur) * (dt / tau) : (tgt - cur);
    const double dv_lim = std::max(0.0, a_max) * dt;
    if (dv_des >  dv_lim) dv_des = dv_lim;
    if (dv_des < -dv_lim) dv_des = -dv_lim;
    return cur + dv_des;
  }

  // 參數
  int period_ms_{20}, cmd_timeout_ms_{500}, log_throttle_ms_{200};
  bool allow_holonomic_{true};
  std::string cmd_topic_{"/robot/cmd_vel"}, odom_topic_{"/robot/pose"};
  std::string frame_id_{"odom"}, child_frame_id_{"base_link"};
  double v_lin_max_{1.2}, v_ang_max_{2.5};
  double a_lin_max_{3.0}, a_ang_max_{6.0};
  double tau_lin_{0.15}, tau_ang_{0.10};

  // 狀態
  double x_{0.0}, y_{0.0}, yaw_{0.0};
  double vx_{0.0}, vy_{0.0}, wz_{0.0};

  // 指令
  double cmd_vx_{0.0}, cmd_vy_{0.0}, cmd_wz_{0.0};
  rclcpp::Time last_cmd_time_{rclcpp::Time(0)};

  // ROS
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time last_step_time_;
};
