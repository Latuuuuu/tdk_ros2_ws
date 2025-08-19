#pragma once
#include "main-pkg/topic.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"  // ⭐ 需要這個

class SpeedPublisher final : public TimerPublisher<geometry_msgs::msg::Twist> {
public:
  SpeedPublisher()
  : TimerPublisher<geometry_msgs::msg::Twist>(
      "speed_publisher", "cmd_vel", rclcpp::QoS(10), std::chrono::milliseconds(100)) 
  {
    // 參數：速度 + 頻率 + 日誌節流（ms；0=每次都印）
    vx_ = this->declare_parameter<double>("vx", 0.5);
    vy_ = this->declare_parameter<double>("vy", 0.0);
    vw_ = this->declare_parameter<double>("vw", 0.1);
    period_ms_ = this->declare_parameter<int>("period_ms", 100);
    log_throttle_ms_ = this->declare_parameter<int>("log_throttle_ms", 0);

    // 依參數調整頻率
    this->set_period(std::chrono::milliseconds(period_ms_));

    // 支援動態調參（vx/vy/vw/period_ms/log_throttle_ms）
    cb_handle_ = this->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params){
        for (const auto & p : params) {
          if (p.get_name() == "vx") vx_ = p.as_double();
          else if (p.get_name() == "vy") vy_ = p.as_double();
          else if (p.get_name() == "vw") vw_ = p.as_double();
          else if (p.get_name() == "period_ms") {
            period_ms_ = p.as_int();
            this->set_period(std::chrono::milliseconds(period_ms_));
          } else if (p.get_name() == "log_throttle_ms") {
            log_throttle_ms_ = p.as_int();
          }
        }
        // 改參後印一次目前設定
        RCLCPP_INFO(this->get_logger(),
          "[cmd_vel param] vx=%.3f vy=%.3f vw=%.3f period=%dms throttle=%dms",
          vx_, vy_, vw_, period_ms_, log_throttle_ms_);
        rcl_interfaces::msg::SetParametersResult res;
        res.successful = true;
        return res;
      });
  }

protected:
  geometry_msgs::msg::Twist make_message() override {
    geometry_msgs::msg::Twist msg;
    msg.linear.x  = vx_;
    msg.linear.y  = vy_;
    msg.angular.z = vw_;

    // ✅ 終端機輸出（可節流）
    if (log_throttle_ms_ > 0) {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), static_cast<uint64_t>(log_throttle_ms_),
        "[cmd_vel] vx=%.3f vy=%.3f vw=%.3f", vx_, vy_, vw_);
    } else {
      RCLCPP_INFO(this->get_logger(),
        "[cmd_vel] vx=%.3f vy=%.3f vw=%.3f", vx_, vy_, vw_);
    }

    return msg;
  }

private:
  double vx_, vy_, vw_;
  int period_ms_, log_throttle_ms_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr cb_handle_;
};
