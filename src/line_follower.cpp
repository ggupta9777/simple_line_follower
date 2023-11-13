#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "nav_msgs/msg/odometry.hpp"
#include "yaml-cpp/yaml.h"
#include <cmath>

class LineFollower : public rclcpp::Node {
public:
  LineFollower() : Node("line_follower") {
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    load_waypoints();
    control_loop_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&LineFollower::controlLoop, this));
  }

  void load_waypoints() {
    // Here you would load waypoints from a YAML file.
    // For simplicity, waypoints are hardcoded here.
    waypoint_1_.x = 1.0; waypoint_1_.y = 0.0; waypoint_1_.z = 0.0;
    waypoint_2_.x = 5.0; waypoint_2_.y = 0.0; waypoint_2_.z = 0.0;
  }

  // Extracted function to calculate error for unit testing.
  double calculateError(const geometry_msgs::msg::TransformStamped& transform) {
    double goal_x = waypoint_2_.x - waypoint_1_.x;
    double goal_y = waypoint_2_.y - waypoint_1_.y;
    
    return (goal_y * transform.transform.translation.x -
            goal_x * transform.transform.translation.y +
            waypoint_2_.x * waypoint_1_.y -
            waypoint_2_.y * waypoint_1_.x) /
            std::sqrt(std::pow(goal_y, 2) + std::pow(goal_x, 2));
  }

  // Function to calculate the control effort from the error.
  double calculateControlEffort(double error) {
    double k_p = 1.0; // Proportional gain
    return k_p * error;
  }

  // Function to create and publish the Twist message.
  void publishVelocity(double linear_x, double angular_z) {
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = linear_x;
    cmd_vel.angular.z = angular_z;
    cmd_vel_pub_->publish(cmd_vel);
  }

private:
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr control_loop_timer_;
  geometry_msgs::msg::Point waypoint_1_;
  geometry_msgs::msg::Point waypoint_2_;

  void controlLoop() {
    geometry_msgs::msg::TransformStamped transformStamped;
    try {
      transformStamped = tf_buffer_.lookupTransform("base_link", "odom", tf2::TimePointZero);
      double error = calculateError(transformStamped);
      double control_effort = calculateControlEffort(error);
      publishVelocity(0.5, -control_effort); // constant forward velocity and dynamic angular velocity
    } catch (const tf2::TransformException &ex) {
      RCLCPP_ERROR(this->get_logger(), "Could not transform %s to %s: %s",
                   "base_link", "odom", ex.what());
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LineFollower>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
