#include <chrono>
#include <memory>
#include <map>
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include "rospi_pre/srv/chassis_manager.hpp"

using namespace std::chrono_literals;

// Utility functions - keep these outside the class since they don't depend on class state
float dist(const geometry_msgs::msg::Point &point1, const geometry_msgs::msg::Point &point2) {
    return std::sqrt(std::pow(point2.x - point1.x, 2) +
                     std::pow(point2.y - point1.y, 2));
}

float linear_velo_magnitude(const geometry_msgs::msg::Twist data) {
    return std::sqrt(std::pow(data.linear.x, 2) +
                     std::pow(data.linear.y, 2));
}

float angular_velo_magnitude(const geometry_msgs::msg::Twist data) {
    return data.angular.z;
}

// Unified node class
class ChassisController : public rclcpp::Node {
public:
  ChassisController() : Node("chassis_controller") {
    // Initialize subscription (from position_subscriber)
    pose_subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
      "pose_feedback", 10, std::bind(&ChassisController::pose_callback, this, std::placeholders::_1));
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr position_subscriber_;
    rclcpp::Service<geometry_msgs::msg::PoseStamped>::SharedPtr goal_server_;
    // Initialize publisher (from twist_publisher)
    twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    
    // Initialize service (from target_pose_handler)
    chassis_service_ = this->create_service<rospi_pre::srv::ChassisManager>(
      "chassis_goal_update", std::bind(&ChassisController::chassis_callback, this, 
                                       std::placeholders::_1, std::placeholders::_2));
    
    // Initialize timer (from twist_publisher)
    timer_ = this->create_wall_timer(50ms, std::bind(&ChassisController::timer_callback, this));
    
    // Set up position data (moved from global position_ds class)
    setup_position_data();
    
    RCLCPP_INFO(this->get_logger(), "Chassis controller initialized");
  }

private:
  // Current robot pose
  geometry_msgs::msg::Pose current_pose_;
  // Twist message to be published
  geometry_msgs::msg::Twist twist_msg_;
  
  // These were previously global variables
  geometry_msgs::msg::Pose cur_origin_;
  geometry_msgs::msg::Pose cur_goal_;
  char cur_move_type_ = 'l'; // Default to linear movement
  
  // Position data (previously in position_ds class)
  std::map<std::string, geometry_msgs::msg::Pose> position_map_;
  
  // ROS components
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;
  rclcpp::Service<rospi_pre::srv::ChassisManager>::SharedPtr chassis_service_;
  rclcpp::TimerBase::SharedPtr timer_;
  double dt = 0.05; // refer to timer init
  
  // Setup position data (previously in position_ds::setup_str_to_pose)
  void setup_position_data() {
    position_map_["init"] = create_pose(0, 0, M_PI/2);
    position_map_["1_finish"] = create_pose(0, 578, M_PI/2);
    position_map_["4_1_curve_start"] = create_pose(0, 0, 0);
    position_map_["4_1_curve_end"] = create_pose(60, 60, M_PI/2);
    position_map_["4_1_curve_end"].position.z = 1.0;
    position_map_["4_1_curve_end"].orientation.x = 60.0; // radius
    position_map_["4_1_curve_end"].orientation.y = 1.0; // direction
    position_map_["4_2_curve_start"] = create_pose(60, 175, M_PI/2);
    position_map_["4_2_curve_end"] = create_pose(160, 175, M_PI*3/2);
    position_map_["4_2_curve_end"].position.z = -1;
    position_map_["4_2_curve_end"].orientation.x = 50;
    position_map_["4_2_curve_end"].orientation.y = 1;
    position_map_["4_3_curve_start"] = create_pose(160, -70, M_PI*3/2);
    position_map_["4_3_curve_end"] = create_pose(260, -70, M_PI/2);
    position_map_["4_3_curve_end"].position.z = 1;
    position_map_["4_3_curve_end"].orientation.x = 50;
    position_map_["4_3_curve_end"].orientation.y = 1;
    
    // Set initial goal
    cur_goal_ = position_map_["init"];
  }
  
  // Helper to create pose (previously in position_ds)
  geometry_msgs::msg::Pose create_pose(float x, float y, float theta) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = 0;
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = theta;
    pose.orientation.w = 0;
    return pose;
  }
  static geometry_msgs::msg::Pose create_circular(double px, double py, double pz, double ox, double oy, double oz, double ow ) {
      geometry_msgs::msg::Pose pose;
      pose.position.x = px;
      pose.position.y = py;
      pose.position.z = pz;
      pose.orientation.x = ox;
      pose.orientation.y = oy;
      pose.orientation.z = oz;
      pose.orientation.w = ow; 
      return pose;
  }
  
  // Callback for pose subscription
  void pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg) {
    // Store the current pose
    current_pose_.position.x = msg->position.x;
    current_pose_.position.y = msg->position.y;
    current_pose_.position.z = msg->position.z;
    current_pose_.orientation.x = msg->orientation.x;
    current_pose_.orientation.y = msg->orientation.y;
    current_pose_.orientation.z = msg->orientation.z;
    current_pose_.orientation.w = msg->orientation.w;

    RCLCPP_INFO(this->get_logger(), 
                "Received Position: position(x: %.2f, y: %.2f, z: %.2f), orientation(z: %.2f)",
                msg->position.x, msg->position.y, msg->position.z, msg->orientation.z);
  }
  
  // Callback for service requests
  void chassis_callback(const std::shared_ptr<rospi_pre::srv::ChassisManager::Request> request,
                       std::shared_ptr<rospi_pre::srv::ChassisManager::Response> response) {
    bool key_valid = position_map_.find(request->command) != position_map_.end();
    response->valid = key_valid;
    
    if (key_valid) {
      cur_origin_ = cur_goal_;
      cur_goal_ = position_map_[request->command];
      cur_move_type_ = request->move_type;
      
      RCLCPP_INFO(this->get_logger(), 
                "Setting goal: position(x: %.2f, y: %.2f), mode: %c",
                cur_goal_.position.x, cur_goal_.position.y, cur_move_type_);
    }
  }
  
  // Timer callback to update and publish twist messages
  void timer_callback() {
    update_twist(twist_msg_);
    
    RCLCPP_INFO(this->get_logger(), 
                "Publishing Twist: linear(x: %.2f, y: %.2f), angular(z: %.2f)",
                twist_msg_.linear.x, twist_msg_.linear.y, twist_msg_.angular.z);
                
    twist_publisher_->publish(twist_msg_);
  }
  
  // Reset all twist values to zero
  void reset_twist(geometry_msgs::msg::Twist &msg) {
    msg.linear.x = 0;
    msg.linear.y = 0;
    msg.linear.z = 0;
    msg.angular.x = 0;
    msg.angular.y = 0;
    msg.angular.z = 0;
  }
  
  // Update twist based on current position and goal
  void update_twist(geometry_msgs::msg::Twist &msg) {
    // Use current_pose_ instead of calling position_subscriber::get_pose()
    geometry_msgs::msg::Pose &cur_pose = current_pose_;
    
    // Linear movement
    if (cur_move_type_ == 'l') {
      // Parameters
      double linear_accel_dist, linear_decel_dist;
      double linear_max_speed = 40, linear_min_speed = 15;
      double linear_acceler = 6, linear_deceler = 6; // cm/s
      double linear_acceler_real = linear_acceler * dt;
      double linear_deceler_real = linear_deceler * dt;
      double linear_error = 2;

      linear_accel_dist = (pow(linear_max_speed, 2)-pow(linear_min_speed, 2)) / (2 * linear_acceler) * 1.2;
      linear_decel_dist = (pow(linear_max_speed, 2)-pow(linear_min_speed, 2)) / (2 * linear_deceler) * 1.2;

      // Check if we've reached the goal
      if (dist(cur_pose.position, cur_goal_.position) < linear_error) {
        reset_twist(msg);
        return;
      }

      // Calculate distances
      double total_dist = dist(cur_goal_.position, cur_origin_.position);
      double travel_dist = dist(cur_pose.position, cur_origin_.position);
      double remain_dist = dist(cur_goal_.position, cur_pose.position);
      double velo_magnitude = linear_velo_magnitude(msg);

      // Acceleration and deceleration logic
      if (total_dist - travel_dist < linear_decel_dist) {
        if (velo_magnitude - linear_deceler > linear_min_speed) {
          velo_magnitude -= linear_deceler_real;
        } else {
          velo_magnitude = linear_min_speed;
        }
      } else {
        if (travel_dist < linear_accel_dist) {
          velo_magnitude += linear_acceler_real;
        } else {
          velo_magnitude = linear_max_speed;
        }
      }

      // Transform velocity to robot frame
      double direction_x = 0, direction_y = 0;
      if (remain_dist > 0) {
        double dx = cur_goal_.position.x - cur_pose.position.x;
        double dy = cur_goal_.position.y - cur_pose.position.y;
        double th = -1 * cur_pose.orientation.z;
        double trans_x = cos(th)*dx - sin(th)*dy;
        double trans_y = sin(th)*dx + cos(th)*dy;
        direction_x = trans_x / remain_dist;
        direction_y = trans_y / remain_dist;
      }

      // Set twist values
      msg.linear.x = velo_magnitude * direction_x;
      msg.linear.y = velo_magnitude * direction_y;
      msg.angular.z = 0;
    }
    // Angular movement
    else if (cur_move_type_ == 'a') {
      double angular_error = 0.017 * 5;

      // Check if we've reached the goal orientation
      if (std::abs(cur_goal_.orientation.z - cur_pose.orientation.z) < angular_error) {
        reset_twist(msg);
        return;
      }

      // Set angular velocity
      double omega_magnitude = 0.35;
      double angular_direction = (cur_goal_.orientation.z > cur_pose.orientation.z) ? 1.0 : -1.0;
      
      msg.linear.x = 0;
      msg.linear.y = 0;
      msg.angular.z = omega_magnitude * angular_direction;
    }
    // Circular movement
    else if (cur_move_type_ == 'c') {
      double circular_error = 4;

      // Check if we've reached the goal position
      if (dist(cur_pose.position, cur_goal_.position) < circular_error) {
        reset_twist(msg);
        return;
      }

    RCLCPP_INFO(this->get_logger(), 
                "circular remaining dist:%.2f", dist(cur_pose.position, cur_goal_.position));

      // Set circular motion parameters
      int direction = cur_goal_.position.z; // -1 or 1
      double radius = cur_goal_.orientation.x;
      double T = 15;
      double omega = 2.0*M_PI / T;
      
      msg.linear.x = radius * omega;
      msg.angular.z = direction * omega;
    }
  }
};

int main(int argc, char * argv[])
{
  // Initialize ROS
  rclcpp::init(argc, argv);
  
  // Create and spin the unified node
  auto node = std::make_shared<ChassisController>();
  
  // Spin the node
  rclcpp::spin(node);
  
  // Clean up
  rclcpp::shutdown();
  return 0;
}
    
