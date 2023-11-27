#include "simple_line_follower/line_follower.hpp"

LineFollower::LineFollower()
: Node("line_follower")
{
  // Initialize publisher to cmd_vel
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("bcr_bot/cmd_vel", 1);

  // Initialize subscriber to odom
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/bcr_bot/odom", 1, 
    std::bind(&LineFollower::odomCb, this, std::placeholders::_1));

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

  // Create a timer to call the controlLoop
  control_loop_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50),
    std::bind(&LineFollower::controlLoop, this));
}

// Callback for odom topic
void LineFollower::odomCb(nav_msgs::msg::Odometry::SharedPtr msg)
{
  bot_x = msg->pose.pose.position.x;
  bot_y = msg->pose.pose.position.y;
}

// Function to Load the ROS 2 Param to get gains
void LineFollower::setControlGain()
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
void LineFollower::loadWaypoints()
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

// Function to calculate error while following the line
double LineFollower::calculateDistanceError()
{
  double goal_x = waypoint_2_.x - waypoint_1_.x;
  double goal_y = waypoint_2_.y - waypoint_1_.y;
  
  // Distance between a point and a line
  double error = (goal_y * bot_x -
    goal_x * bot_y +
    waypoint_2_.x * waypoint_1_.y -
    waypoint_2_.y * waypoint_1_.x) /
    std::sqrt(std::pow(goal_y, 2) + std::pow(goal_x, 2));

  return error;
}

// Function to calculate the proportional control effort
double LineFollower::calculateProportionalEffort(double error)
{ 
  // Calculate proportional effort
  double prop_effort = k_p*error; 

  return prop_effort;
}

// Function to calculate the derivative control effort
double LineFollower::calculateDerivativeEffort(double error)
{ 
  // Calculating difference in errors for the derivative controller
  double error_diff = (error - prev_error);
  
  // Calculate effort
  double der_effort = k_d*error_diff;

  // Update value of prev_error
  prev_error = error ;

  return der_effort;
}

// Function to create and publish the Twist message
void LineFollower::publishVelocity(double linear_x, double angular_z)
{ 
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = linear_x;
  cmd_vel.angular.z = angular_z;
  cmd_vel_pub_->publish(cmd_vel);
}

// Main control loop  
void LineFollower::controlLoop()
{   
  // Calculate distance between bot and the line
  double distance_error = calculateDistanceError();

  // Calculate the effort required to minimize the distance_error 
  double control_effort = calculateProportionalEffort(distance_error) + calculateDerivativeEffort(distance_error);

  // Saturate effort between -10 and 10
  control_effort = std::min(std::max(control_effort, -10.0), 10.0);
  
  // Publish Velocity messages
  publishVelocity(linear_vel, control_effort);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LineFollower>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
