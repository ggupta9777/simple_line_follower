#include "simple_line_follower/naive_line_follower.hpp"

NaiveLineFollower::NaiveLineFollower()
: Node("naive_line_follower")
{
  // Initialize publisher
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("bcr_bot/cmd_vel", 1);

  // Initialize subscriber to odom
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/bcr_bot/odom", 1, 
    std::bind(&NaiveLineFollower::odomCb, this, std::placeholders::_1));

  // Declare ROS 2 Param to get waypoints and gains
  this->declare_parameter("k_p", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("k_d", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("waypoints", rclcpp::PARAMETER_DOUBLE_ARRAY);

  // Set constant linear velocity and gains
  linear_vel = 1.0;
  k_p = 1.0;
  k_d = 20.0;

  // Load ROS 2 Params
  setControlGain();
  loadWaypoints();

  // Create a timer to call goToGoal
  control_loop_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50),
    std::bind(&NaiveLineFollower::goToGoal, this));
}

// Callback for odom topic
void NaiveLineFollower::odomCb(nav_msgs::msg::Odometry::SharedPtr msg)
{
  bot_x = msg->pose.pose.position.x;
  bot_y = msg->pose.pose.position.y;
}

// Function to Load the ROS 2 Param to get gains
void NaiveLineFollower::setControlGain()
{
  try
  {
    k_p = this->get_parameter("k_p").as_double();    
    RCLCPP_INFO(this->get_logger(), "Proportional Gain set successfully : %f", k_p);    
  }
  catch (const std::exception& e)
  {
    RCLCPP_INFO(this->get_logger(), "Exception caught: %s", e.what());
  }
  
  try
  {
    k_d = this->get_parameter("k_d").as_double();
    RCLCPP_INFO(this->get_logger(), "Derivative Gain set successfully : %f", k_d);
  }
  catch (const std::exception& e)
  {
    RCLCPP_INFO(this->get_logger(), "Exception caught: %s", e.what());
  }
}

// Function to Load the ROS 2 Param to get waypoints
void NaiveLineFollower::loadWaypoints()
{
  try
  {
    auto waypoints_param = this->get_parameter("waypoints");
  
    // Converting waypoints_param to a vector<double>
    waypoints = waypoints_param.as_double_array();
    waypoint_1_.x = waypoints[0]; waypoint_1_.y = waypoints[1]; waypoint_1_.z = waypoints[2];
    waypoint_2_.x = waypoints[3]; waypoint_2_.y = waypoints[4]; waypoint_2_.z = waypoints[5];

    RCLCPP_INFO(this->get_logger(), "Waypoints set successfully");
  }
  catch (const std::exception& e)
  {        
    RCLCPP_INFO(this->get_logger(), "Exception caught: %s", e.what());
  }    
}

void NaiveLineFollower::goToGoal()
{  
  // Calculate error while following the line
  double goal_x = waypoint_2_.x - waypoint_1_.x;
  double goal_y = waypoint_2_.y - waypoint_1_.y;
  
  // Distance between a point and a line
  double error = (goal_y * bot_x -
    goal_x * bot_y +
    waypoint_2_.x * waypoint_1_.y -
    waypoint_2_.y * waypoint_1_.x) /
    std::sqrt(std::pow(goal_y, 2) + std::pow(goal_x, 2));
  
  // Calculating difference in errors for the derivative controller
  double error_diff = (error - prev_error);
  
  // Calculate effort
  double effort = k_p*error + k_d*error_diff;
  
  // Update value of prev_error
  prev_error = error ;
  
  // Saturate effort between -10 and 10
  effort = std::min(std::max(effort, -10.0), 10.0);
  
  // Create and publish the Twist message
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel_.linear.x = linear_vel;
  cmd_vel_.angular.z = effort; 
  cmd_vel_pub_->publish(cmd_vel_);   
}
  
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NaiveLineFollower>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
