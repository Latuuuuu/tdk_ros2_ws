#pragma once
// TODO: include message and service type for the service

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
// #include "interfaces/srv/distance.hpp" 

using Distance = interfaces::srv::Distance;


class Server : public rclcpp::Node{

  public:
    Server();

  private:
    // TODO: define the callback function for the server
    void distance_callback(const std::shared_ptr<Distance::Request> request,std::shared_ptr<Distance::Response> response);
    
    rclcpp::Service<Distance>::SharedPtr service_;
};

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