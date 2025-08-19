#pragma once
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>

template<typename MsgT>
class TimerPublisher : public rclcpp::Node {    //繼承 rclcpp::Node，成為一個 ROS 2 節點
public:
  using Duration = std::chrono::milliseconds;    //相當於 typedef std::chrono::milliseconds Duration;

  TimerPublisher(const std::string & node_name,    //建構子
                 const std::string & topic_name,
                 const rclcpp::QoS & qos,    //quality of service，處理消息傳播的可靠性、延遲、寬帶
                 Duration period)
  : rclcpp::Node(node_name) {
    publisher_ = this->create_publisher<MsgT>(topic_name, qos);
    timer_ = this->create_wall_timer(period, std::bind(&TimerPublisher::on_timer, this));
  }

protected:
  virtual MsgT make_message() = 0;    // 宣告一個純虛函式強迫子類別必須實作發佈訊息 MsgT

  void publish_now() {
    publisher_->publish(make_message());    //負責publish
  }

  void set_period(Duration period) {    //動態調整頻率（非必要）
    timer_ = this->create_wall_timer(period, std::bind(&TimerPublisher::on_timer, this));    // 重新建立 timer（舊的會被替換掉）
  }

private:
  void on_timer() { publish_now(); }    //計時器呼叫

protected:
  typename rclcpp::Publisher<MsgT>::SharedPtr publisher_;    //Publisher 與 Timer 的 shared_ptr
  typename rclcpp::TimerBase::SharedPtr timer_;
};


// // 你原本的 sub 先保留（目前沒用到）
// class sub : public rclcpp::Node{
// public:
//   sub();
// private:
//   void topic_callback(const std_msgs::msg::Int64::SharedPtr msg);
//   rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr subscription_;
// };

// class GenericSubscriber : public rclcpp::Node {
// public:
//   using Callback = std::function<void(const std::shared_ptr<const MsgT>&)>;

//   GenericSubscriber(const std::string& node_name,
//                     const std::string& topic_name,
//                     const rclcpp::QoS& qos,
//                     Callback cb = nullptr)
//   : rclcpp::Node(node_name),
//     topic_name_(topic_name),
//     qos_(qos),
//     user_cb_(std::move(cb))   // 使用者自訂回呼（可空）
//   {
//     subscribe_();             // 立刻建立訂閱
//   }

//   // 允許動態換 topic（會重新建立 subscription）
//   void set_topic(const std::string& topic_name) {
//     topic_name_ = topic_name;
//     subscribe_();
//   }

//   // 允許動態換 QoS（會重新建立 subscription）
//   void set_qos(const rclcpp::QoS& qos) {
//     qos_ = qos;
//     subscribe_();
//   }

//   // 允許動態換使用者回呼（不必繼承也能客製行為）
//   void set_callback(Callback cb) {
//     user_cb_ = std::move(cb);
//   }

// protected:
//   // 子類別可覆寫這個函式處理訊息（若未覆寫而且也沒設 user_cb_，預設不做事）
//   virtual void on_message(const std::shared_ptr<const MsgT>& /*msg*/) {}

// private:
//   void subscribe_() {
//     // 每次（re）subscribe 時產生新 lambda，把 user_cb_ 和 on_message 串成一個回呼
//     auto wrapper = [this](const std::shared_ptr<const MsgT> msg) {
//       if (user_cb_) {
//         user_cb_(msg);        // 使用者傳入的回呼（優先）
//       } else {
//         this->on_message(msg); // 或使用繼承覆寫的處理函式
//       }
//     };
//     subscription_ = this->create_subscription<MsgT>(topic_name_, qos_, wrapper);
//   }
// };