#pragma once
#include "main-pkg/topic.hpp"
#include "geometry_msgs/msg/twist.hpp"  // 要發佈的速度型態
#include "geometry_msgs/msg/pose_stamped.hpp" // 目標位置型態
#include "nav_msgs/msg/odometry.hpp"  // 狀態型態
#include "rcl_interfaces/msg/set_parameters_result.hpp" // 動態參數回呼結果型態
#include "visualization_msgs/msg/marker.hpp"  // 用於可視化
#include "visualization_msgs/msg/marker_array.hpp"   
#include "nav_msgs/msg/path.hpp"  
#include <cmath>  
#include <algorithm>

class SpeedPublisher final : public TimerPublisher<geometry_msgs::msg::Twist> { // 繼承 TimerPublisher，發佈速度訊息，final表示這個類別不能被繼承
public:
  SpeedPublisher()
  : TimerPublisher<geometry_msgs::msg::Twist>(
      "speed_publisher", "/robot/cmd_vel", rclcpp::QoS(10), std::chrono::milliseconds(50))
  {
    // ===== 參數 =====
    period_ms_       = this->declare_parameter<int>("period_ms", 50); // 發佈頻率（毫秒）
    log_throttle_ms_ = this->declare_parameter<int>("log_throttle_ms", 200);  // terminal輸出節流（毫秒）
    print_csv_       = this->declare_parameter<bool>("print_csv", false); // 是否輸出 CSV 格式

    // 速度 / 加速度上限
    v_lin_max_ = this->declare_parameter<double>("v_lin_max", 0.8);   // m/s
    v_ang_max_ = this->declare_parameter<double>("v_ang_max", 1.5);   // rad/s
    a_lin_max_ = this->declare_parameter<double>("a_lin_max", 1.0);   // m/s^2
    a_ang_max_ = this->declare_parameter<double>("a_ang_max", 2.0);   // rad/s^2

    // 控制增益與收斂判斷
    kp_lin_ = this->declare_parameter<double>("kp_lin", 1.5);
    kp_ang_ = this->declare_parameter<double>("kp_ang", 2.5);
    kd_ang_ = this->declare_parameter<double>("kd_ang", 0.2); // 用 odom 角速作阻尼
    pos_tol_  = this->declare_parameter<double>("pos_tolerance", 0.2);  // m
    yaw_tol_  = this->declare_parameter<double>("yaw_tolerance", 0.03);  // rad

    // 是否允許側向速度（麥克納姆/全向底盤用）
    allow_holonomic_ = this->declare_parameter<bool>("allow_holonomic", true);

    // topic 名稱可用參數改
    state_topic_ = this->declare_parameter<std::string>("state_topic", "/robot/pose");
    goal_topic_  = this->declare_parameter<std::string>("goal_topic",  "/goal");

    // 目標方向相關參數
    use_goal_orientation_     = this->declare_parameter<bool>("use_goal_orientation", false);
    turn_in_place_before_move_= this->declare_parameter<bool>("turn_in_place_before_move", false);
    rotate_only_              = this->declare_parameter<bool>("rotate_only", false);

    // ===== 視覺化參數 =====
    viz_enable_      = this->declare_parameter<bool>("viz_enable", true);
    viz_frame_       = this->declare_parameter<std::string>("viz_frame", "odom"); // RViz Fixed Frame
    viz_vel_scale_m_ = this->declare_parameter<double>("viz_vel_scale_m", 0.5);   // 速度箭頭長度縮放（公尺/每 m/s）
    viz_pose_scale_m_= this->declare_parameter<double>("viz_pose_scale_m", 0.3);  // 姿態箭頭固定長度（公尺）
    viz_lifetime_ms_ = this->declare_parameter<int>("viz_lifetime_ms", 0);        // 0=永久

    // ===== 視覺化 Publisher =====
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/viz/speed_publisher/markers", 1);
    path_pub_   = this->create_publisher<nav_msgs::msg::Path>("/viz/speed_publisher/path", 1);
    path_.header.frame_id = viz_frame_;
    path_max_len_ = this->declare_parameter<int>("viz_path_max_len", 1000); // 最多保留幾個點

    // 依頻率調整 timer
    this->set_period(std::chrono::milliseconds(period_ms_));

    void publish_viz_(const geometry_msgs::msg::Twist& cmd,double vx_w, double vy_w,double dist, double yaw_err);

    // ===== 訂閱 =====
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      state_topic_, 20,
      [this](nav_msgs::msg::Odometry::SharedPtr msg){
        x_ = msg->pose.pose.position.x;
        y_ = msg->pose.pose.position.y;
        yaw_ = quat_to_yaw(msg->pose.pose.orientation); //quat_to_yaw()將四位元數轉換為歐拉角的偏航角(yaw)

        vx_meas_ = msg->twist.twist.linear.x;
        vy_meas_ = msg->twist.twist.linear.y;
        wz_meas_ = msg->twist.twist.angular.z;

        have_state_ = true;

        if (!msg->header.frame_id.empty()) {
          // 讓可視化自動跟隨 odom 的座標系
          viz_frame_ = msg->header.frame_id;
          path_.header.frame_id = viz_frame_;
        }
      });

    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      goal_topic_, 10,
      [this](geometry_msgs::msg::PoseStamped::SharedPtr msg){
        goal_x_ = msg->pose.position.x;
        goal_y_ = msg->pose.position.y;
        goal_yaw_ = quat_to_yaw(msg->pose.orientation);
        have_goal_ = true;
      });

    // ===== 動態參數回呼 =====
    cb_handle_ = this->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter>& ps){
        for (auto &p : ps) {
          auto n = p.get_name();
          if      (n=="period_ms"){ period_ms_=p.as_int(); this->set_period(std::chrono::milliseconds(period_ms_)); }
          else if (n=="log_throttle_ms") log_throttle_ms_ = p.as_int();
          else if (n=="print_csv")       print_csv_ = p.as_bool();
          else if (n=="v_lin_max") v_lin_max_ = p.as_double();
          else if (n=="v_ang_max") v_ang_max_ = p.as_double();
          else if (n=="a_lin_max") a_lin_max_ = p.as_double();
          else if (n=="a_ang_max") a_ang_max_ = p.as_double();
          else if (n=="kp_lin")    kp_lin_ = p.as_double();
          else if (n=="kp_ang")    kp_ang_ = p.as_double();
          else if (n=="kd_ang")    kd_ang_ = p.as_double();
          else if (n=="pos_tolerance") pos_tol_ = p.as_double();
          else if (n=="yaw_tolerance") yaw_tol_ = p.as_double();
          else if (n=="allow_holonomic") allow_holonomic_ = p.as_bool();
          else if (n=="use_goal_orientation")      use_goal_orientation_ = p.as_bool();
          else if (n=="turn_in_place_before_move") turn_in_place_before_move_ = p.as_bool();
          else if (n=="rotate_only")               rotate_only_ = p.as_bool();
        }
        rcl_interfaces::msg::SetParametersResult r; r.successful = true; return r;
      });

    last_time_ = this->now();
  }

protected:
  geometry_msgs::msg::Twist make_message() override {
    geometry_msgs::msg::Twist cmd;

    // 沒狀態或沒目標 → 停止並提示待命
    if (!have_state_ || !have_goal_) {
      if (log_throttle_ms_ > 0) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(),
          static_cast<uint64_t>(log_throttle_ms_), "[idle] waiting for %s and %s ...",
          have_state_? "goal":"state", have_goal_? "": "goal");
      }
      return cmd;
    }

    // 步長
    const auto now = this->now();
    double dt = (now - last_time_).seconds();
    if (dt <= 0.0) dt = static_cast<double>(period_ms_) / 1000.0;
    last_time_ = now;

    // 誤差
    const double dx = goal_x_ - x_;
    const double dy = goal_y_ - y_;
    const double dist = std::hypot(dx, dy);
    // const double target_heading = std::atan2(dy, dx); // 目標方向
    // double yaw_err = ang_norm(target_heading - yaw_);

    // 兩種「目標朝向」：移動時面向目標點；到點後若啟用則面向 /goal 的 yaw
    const double yaw_target_move  = std::atan2(dy, dx);
    const double yaw_target_final = use_goal_orientation_ ? goal_yaw_ : yaw_target_move;

    // 收斂處理（平滑回 0）
    const bool pos_ok = dist <= pos_tol_;
    // const bool yaw_ok = std::abs(yaw_err) <= yaw_tol_;
    // if (pos_ok && yaw_ok) {
    //   cur_v_lin_ = approach_with_slew(cur_v_lin_, 0.0, a_lin_max_, dt);
    //   cur_w_     = approach_with_slew(cur_w_,     0.0, a_ang_max_, dt);
    //   cmd.linear.x = cmd.linear.y = 0.0;
    //   cmd.angular.z = 0.0;
    //   // 印當前狀態
    //   log_status_(cmd, dist, yaw_err);
    //   return cmd;
    // }

    // 需要先對準再走？（方法二：turn-in-place）
    const double yaw_err_align = ang_norm(yaw_target_move - yaw_);
    const bool need_align_before_move =
      turn_in_place_before_move_ && !pos_ok && std::abs(yaw_err_align) > yaw_tol_;

    // 到點後是否還需要對準最終朝向？（方法一：/goal 的 yaw）
    const double yaw_err_final = ang_norm(yaw_target_final - yaw_);
    const bool need_final_turn = pos_ok && (std::abs(yaw_err_final) > yaw_tol_);

    // 純轉向模式？（方法二：rotate_only）
    const bool rotate_only_now = rotate_only_;

    // 根據情境選擇這一拍要追的「目標 yaw」
    const double yaw_target = (rotate_only_now || need_align_before_move || need_final_turn)
                                ? (need_final_turn ? yaw_target_final : yaw_target_move)
                                : yaw_target_move;
    double yaw_err = ang_norm(yaw_target - yaw_);

    // 角速度：PD + 可停止上限 + 平滑
    const double w_pd   = kp_ang_ * yaw_err - kd_ang_ * wz_meas_;
    const double w_stop = std::sqrt(std::max(0.0, 2.0 * a_ang_max_ * std::max(std::abs(yaw_err) - yaw_tol_, 0.0)));
    double w_raw = std::clamp(w_pd, -v_ang_max_, v_ang_max_);
    w_raw = std::clamp(w_raw, -w_stop, w_stop);
    cur_w_ = approach_with_slew(cur_w_, w_raw, a_ang_max_, dt);

    // 線速度：P + 可停止上限 + 平滑
    bool block_translation = rotate_only_now || need_align_before_move || need_final_turn;
    double v_lin_target = 0.0;
    if (!block_translation) {
      double v_lin_p = std::clamp(kp_lin_ * dist, 0.0, v_lin_max_);
      const double v_stop = std::sqrt(std::max(0.0, 2.0 * a_lin_max_ * std::max(dist - pos_tol_, 0.0)));
      v_lin_target = std::min(v_lin_p, v_stop);
    }
    cur_v_lin_ = approach_with_slew(cur_v_lin_, v_lin_target, a_lin_max_, dt);

    // 指向目標方向（世界座標→本體座標）
    const double ux = (dist > 1e-9) ? (dx / dist) : std::cos(yaw_target_move);
    const double uy = (dist > 1e-9) ? (dy / dist) : std::sin(yaw_target_move);
    const double vx_w = cur_v_lin_ * ux;
    const double vy_w = cur_v_lin_ * uy;
    const double c = std::cos(yaw_), s = std::sin(yaw_);
    const double vx_b =  c * vx_w + s * vy_w;
    const double vy_b = -s * vx_w + c * vy_w;

    cmd.linear.x  = vx_b;
    cmd.linear.y  = allow_holonomic_ ? vy_b : 0.0;
    cmd.angular.z = cur_w_;

    if (viz_enable_) publish_viz_(cmd, vx_w, vy_w, dist, yaw_err);

    //終端輸出（人類可讀 / CSV；可節流）
    log_status_(cmd, dist, yaw_err);
    return cmd;
  }

private:
  // 將 cur 以最大加速度 a_max、時間步 dt，向 tgt 緩變
  static double approach_with_slew(double cur, double tgt, double a_max, double dt) {
    const double max_step = std::max(0.0, a_max) * dt;
    const double d = tgt - cur;
    if (d >  max_step) return cur + max_step;
    if (d < -max_step) return cur - max_step;
    return tgt;
  }
  static double ang_norm(double a) {
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
  }
  static double quat_to_yaw(const geometry_msgs::msg::Quaternion &q) {
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  // ⭐ 新增：統一的終端輸出
  void log_status_(const geometry_msgs::msg::Twist& cmd, double dist, double yaw_err) {
    if (print_csv_) {
      // CSV（建議把 ros2 run 輸出轉檔：... | tee run.csv）
      if (log_throttle_ms_ > 0) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(),
          static_cast<uint64_t>(log_throttle_ms_),
          "CSV, t=%.3f, x=%.3f, y=%.3f, yaw=%.3f, gx=%.3f, gy=%.3f, dist=%.3f, yaw_err=%.3f, "
          "vx_cmd=%.3f, vy_cmd=%.3f, wz_cmd=%.3f, vx_meas=%.3f, vy_meas=%.3f, wz_meas=%.3f",
          this->now().seconds(),
          x_, y_, yaw_, goal_x_, goal_y_, dist, yaw_err,
          cmd.linear.x, cmd.linear.y, cmd.angular.z,
          vx_meas_, vy_meas_, wz_meas_);
      } else {
        RCLCPP_INFO(this->get_logger(),
          "CSV, t=%.3f, x=%.3f, y=%.3f, yaw=%.3f, gx=%.3f, gy=%.3f, dist=%.3f, yaw_err=%.3f, "
          "vx_cmd=%.3f, vy_cmd=%.3f, wz_cmd=%.3f, vx_meas=%.3f, vy_meas=%.3f, wz_meas=%.3f",
          this->now().seconds(),
          x_, y_, yaw_, goal_x_, goal_y_, dist, yaw_err,
          cmd.linear.x, cmd.linear.y, cmd.angular.z,
          vx_meas_, vy_meas_, wz_meas_);
      }
    } else {
      // 人類可讀
      if (log_throttle_ms_ > 0) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(),
          static_cast<uint64_t>(log_throttle_ms_),
          "[ctrl] pos=(%.3f,%.3f) yaw=%.3f | goal=(%.3f,%.3f) dist=%.3f yaw_err=%.3f | "
          "cmd: vx=%.3f vy=%.3f wz=%.3f | meas: vx=%.3f vy=%.3f wz=%.3f",
          x_, y_, yaw_, goal_x_, goal_y_, dist, yaw_err,
          cmd.linear.x, cmd.linear.y, cmd.angular.z,
          vx_meas_, vy_meas_, wz_meas_);
      } else {
        RCLCPP_INFO(this->get_logger(),
          "[ctrl] pos=(%.3f,%.3f) yaw=%.3f | goal=(%.3f,%.3f) dist=%.3f yaw_err=%.3f | "
          "cmd: vx=%.3f vy=%.3f wz=%.3f | meas: vx=%.3f vy=%.3f wz=%.3f",
          x_, y_, yaw_, goal_x_, goal_y_, dist, yaw_err,
          cmd.linear.x, cmd.linear.y, cmd.angular.z,
          vx_meas_, vy_meas_, wz_meas_);
      }
    }
  }

  void publish_viz_(const geometry_msgs::msg::Twist& cmd,
                  double vx_w, double vy_w,
                  double dist, double yaw_err)
{
  visualization_msgs::msg::MarkerArray arr;
  const rclcpp::Time now = this->now();
  const rclcpp::Duration life = (viz_lifetime_ms_ > 0)
      ? rclcpp::Duration::from_seconds(viz_lifetime_ms_ / 1000.0)
      : rclcpp::Duration(0,0);

  auto color = [](float r,float g,float b,float a){
    std_msgs::msg::ColorRGBA c; c.r=r; c.g=g; c.b=b; c.a=a; return c;
  };

  // 0) 目前姿態箭頭（藍色）
  {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = viz_frame_; m.header.stamp = now;
    m.ns = "speed_publisher"; m.id = 0;
    m.type = visualization_msgs::msg::Marker::ARROW;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.lifetime = life;

    geometry_msgs::msg::Point p0, p1;
    p0.x = x_; p0.y = y_; p0.z = 0.05;
    p1.x = x_ + viz_pose_scale_m_ * std::cos(yaw_);
    p1.y = y_ + viz_pose_scale_m_ * std::sin(yaw_);
    p1.z = 0.05;
    m.points = {p0, p1};
    m.scale.x = 0.02; m.scale.y = 0.05; m.scale.z = 0.07;
    m.color = color(0.2f,0.4f,1.0f,0.9f);
    arr.markers.push_back(std::move(m));
  }

  // 1) 速度箭頭（綠色，世界座標方向）
  {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = viz_frame_; m.header.stamp = now;
    m.ns = "speed_publisher"; m.id = 1;
    m.type = visualization_msgs::msg::Marker::ARROW;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.lifetime = life;

    geometry_msgs::msg::Point p0, p1;
    p0.x = x_; p0.y = y_; p0.z = 0.05;
    p1.x = x_ + viz_vel_scale_m_ * vx_w;
    p1.y = y_ + viz_vel_scale_m_ * vy_w;
    p1.z = 0.05;
    m.points = {p0, p1};
    m.scale.x = 0.02; m.scale.y = 0.06; m.scale.z = 0.08;
    m.color = color(0.1f,0.9f,0.2f,0.9f);
    arr.markers.push_back(std::move(m));
  }

  // 2) 目標位置（紅球）
  {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = viz_frame_; m.header.stamp = now;
    m.ns = "speed_publisher"; m.id = 2;
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.lifetime = life;

    m.pose.position.x = goal_x_;
    m.pose.position.y = goal_y_;
    m.pose.position.z = 0.05;
    m.scale.x = m.scale.y = m.scale.z = 0.12;
    m.color = color(1.0f,0.2f,0.2f,0.9f);
    arr.markers.push_back(std::move(m));
  }

  // 3) 目標朝向（橘箭頭；在 use_goal_orientation_ 時顯示）
  if (use_goal_orientation_) {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = viz_frame_; m.header.stamp = now;
    m.ns = "speed_publisher"; m.id = 3;
    m.type = visualization_msgs::msg::Marker::ARROW;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.lifetime = life;

    geometry_msgs::msg::Point p0, p1;
    p0.x = goal_x_; p0.y = goal_y_; p0.z = 0.05;
    p1.x = goal_x_ + 0.3 * std::cos(goal_yaw_);
    p1.y = goal_y_ + 0.3 * std::sin(goal_yaw_);
    p1.z = 0.05;
    m.points = {p0, p1};
    m.scale.x = 0.02; m.scale.y = 0.05; m.scale.z = 0.07;
    m.color = color(1.0f,0.6f,0.1f,0.9f);
    arr.markers.push_back(std::move(m));
  }

  // 4) 路徑（nav_msgs/Path）
  {
    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id = viz_frame_; ps.header.stamp = now;
    ps.pose.position.x = x_; ps.pose.position.y = y_; ps.pose.position.z = 0.0;
    const double cy = std::cos(yaw_*0.5), sy = std::sin(yaw_*0.5);
    ps.pose.orientation.w = cy; ps.pose.orientation.z = sy;
    path_.header.stamp = now;
    path_.poses.push_back(ps);
    if (static_cast<int>(path_.poses.size()) > path_max_len_)
      path_.poses.erase(path_.poses.begin());
  }

  // 2.5) 訊息文字（白色）：顯示 |v|、dist、yaw_err
  {
    const double vmag = std::hypot(cmd.linear.x, cmd.linear.y); // 用到 cmd
    visualization_msgs::msg::Marker m;
    m.header.frame_id = viz_frame_; m.header.stamp = now;
    m.ns = "speed_publisher"; m.id = 25;
    m.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.lifetime = life;

    m.pose.position.x = x_;
    m.pose.position.y = y_;
    m.pose.position.z = 0.35;  // 文字浮在上方一點
    m.scale.z = 0.12;          // 文字高度（公尺）
    m.color = color(1.0f,1.0f,1.0f,0.95f);

    std::ostringstream ss;
    ss.setf(std::ios::fixed); ss.precision(3);
    ss << "|v|=" << vmag << " m/s, dist=" << dist << " m, yaw_err=" << yaw_err << " rad";
    m.text = ss.str();

    arr.markers.push_back(std::move(m));
  }


  marker_pub_->publish(arr);
  path_pub_->publish(path_);
}


  // 參數
  int period_ms_{50}, log_throttle_ms_{200};
  bool  print_csv_{false};
  double v_lin_max_{0.8}, v_ang_max_{1.5};
  double a_lin_max_{1.0}, a_ang_max_{2.0};
  double kp_lin_{1.5}, kp_ang_{2.5}, kd_ang_{0.2};
  double pos_tol_{0.02}, yaw_tol_{0.03};
  bool   allow_holonomic_{true};
  std::string state_topic_{"/robot/pose"}, goal_topic_{"/goal"};

  // 狀態（世界座標）
  double x_{0.0}, y_{0.0}, yaw_{0.0};
  double vx_meas_{0.0}, vy_meas_{0.0}, wz_meas_{0.0};
  bool   have_state_{false};

  // 目標
  double goal_x_{0.0}, goal_y_{0.0}, goal_yaw_{0.0};
  bool   have_goal_{false};

  // 平滑器內部狀態（上一個指令）
  double cur_v_lin_{0.0};
  double cur_w_{0.0};

  // 目標朝向相關參數
  bool use_goal_orientation_{false};
  bool turn_in_place_before_move_{false};
  bool rotate_only_{false};

  // RViz 視覺化
  bool viz_enable_{true};
  std::string viz_frame_{"odom"};
  double viz_vel_scale_m_{0.5};
  double viz_pose_scale_m_{0.3};
  int viz_lifetime_ms_{0};
  int path_max_len_{1000};
  nav_msgs::msg::Path path_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr           path_pub_;

  // 時間
  rclcpp::Time last_time_;

  // ROS
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr cb_handle_;
};
