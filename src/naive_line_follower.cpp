#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "nav_msgs/msg/odometry.hpp"
#include "yaml-cpp/yaml.h"

class NaiveLineFollower : public rclcpp::Node {
public:
  NaiveLineFollower() : Node("naive_line_follower") {
    // Initialize publishers and subscribers
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Load waypoints from the config file
    load_waypoints();

    // Initialize the control loop timer
    control_loop_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&NaiveLineFollower::goToGoal, this));
  }

private:
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr control_loop_timer_;
  geometry_msgs::msg::Point waypoint_1_;
  geometry_msgs::msg::Point waypoint_2_;
  geometry_msgs::msg::Twist cmd_vel_;

  void load_waypoints() {
    // This function should load waypoints from a YAML file.
    // For simplicity, we are hardcoding the waypoints here.
    waypoint_1_.x = 1.0;
    waypoint_1_.y = 0.0;
    waypoint_1_.z = 0.0;
    waypoint_2_.x = 5.0;
    waypoint_2_.y = 0.0;
    waypoint_2_.z = 0.0;
  }

  void goToGoal() {
    // This function gets called at a fixed time interval to update the robot's movement.
    // For this example, we apply a simple proportional control based on the robot's current position
    // and the line formed by the waypoints.
    geometry_msgs::msg::TransformStamped transformStamped;

    try {
      transformStamped = tf_buffer_.lookupTransform("base_link", "odom", tf2::TimePointZero, std::chrono::milliseconds(100));
      double goal_x = waypoint_2_.x - waypoint_1_.x;
      double goal_y = waypoint_2_.y - waypoint_1_.y;
      
      // Calculate the error term (distance to the line)
      double error = (goal_y * transformStamped.transform.translation.x -
                      goal_x * transformStamped.transform.translation.y +
                      waypoint_2_.x * waypoint_1_.y -
                      waypoint_2_.y * waypoint_1_.x) /
                     std::sqrt(std::pow(goal_y, 2) + std::pow(goal_x, 2));
      
      // Apply proportional control
      double k_p = 1.0; // Proportional gain
      double control_effort = k_p * error;

      // Populate cmd_vel
      cmd_vel_.linear.x = 0.5; // constant forward velocity
      cmd_vel_.angular.z = -control_effort; // turning rate

      // Publish cmd_vel
      cmd_vel_pub_->publish(cmd_vel_);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_ERROR(this->get_logger(), "Could not transform %s to %s: %s",
                   "base_link", "odom", ex.what());
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NaiveLineFollower>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}