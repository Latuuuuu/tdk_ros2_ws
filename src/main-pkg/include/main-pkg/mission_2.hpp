#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"

class Client : public rclcpp::Node{

  public:
    Client();

  private:
    // TODO: define the timer and callback function for the server
    void handle_callback(rclcpp::Client<Distance>::SharedFuture future);
    void timer_callback();
    rclcpp::Client<Distance>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;
    double x_{0.0}, y_{0.0};
    // TODO: create a timer and client for the service
};