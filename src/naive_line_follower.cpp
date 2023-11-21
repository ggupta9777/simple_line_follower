#include "simple_line_follower/naive_line_follower.hpp"

NaiveLineFollower::NaiveLineFollower()
: Node("naive_line_follower"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
{
  // Initialize publisher
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("bcr_bot/cmd_vel", 1);

  // Declare ROS 2 Param to get waypoints and proportional gain
  this->declare_parameter("k_p", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("waypoints", rclcpp::PARAMETER_DOUBLE_ARRAY);

  // Set constant linear velocity and proportional gain
  linear_vel = 1.0;
  k_p = 1.0;

  // Load ROS 2 Params
  setControlGain();
  loadWaypoints();

  // Create a timer to call goToGoal
  control_loop_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50),
    std::bind(&NaiveLineFollower::goToGoal, this));
}

// Function to Load the ROS 2 Param to get proportional gain
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
    waypoint_angle = std::atan2(waypoint_2_.y - waypoint_1_.y, waypoint_2_.x - waypoint_1_.x);

    RCLCPP_INFO(this->get_logger(), "Waypoints set successfully");
  }
  catch (const std::exception& e)
  {        
    RCLCPP_INFO(this->get_logger(), "Exception caught: %s", e.what());
  }    
}

void NaiveLineFollower::goToGoal()
{
  geometry_msgs::msg::TransformStamped transform_stamped;  
  try
  {
    // Lookup transform to get the bot's position
    transform_stamped = tf_buffer_.lookupTransform("odom", "base_link", tf2::TimePointZero, std::chrono::milliseconds(100));       
    
    // Calculate error while following the line
    double goal_x = waypoint_2_.x - waypoint_1_.x;
    double goal_y = waypoint_2_.y - waypoint_1_.y;
    
    // Distance between a point and a line
    distance_error = (goal_y * transform_stamped.transform.translation.x -
      goal_x * transform_stamped.transform.translation.y +
      waypoint_2_.x * waypoint_1_.y -
      waypoint_2_.y * waypoint_1_.x) /
      std::sqrt(std::pow(goal_y, 2) + std::pow(goal_x, 2));

    // Calculate the distance effort from the distance error
    distance_effort = k_p * distance_error;
    // Saturate effort between -100 and 100
    distance_effort = std::min(std::max(distance_effort, -10.0), 10.0);
    
    // Create and publish the Twist message
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel_.linear.x = linear_vel;
    cmd_vel_.angular.z = distance_effort; 
    cmd_vel_pub_->publish(cmd_vel_);
    
    RCLCPP_INFO(this->get_logger(), "Following Line");    
  }
  catch (const tf2::TransformException &ex)
  {
  RCLCPP_ERROR(this->get_logger(), "Could not transform %s to %s: %s", "odom", "base_link", ex.what());
  }
}
  
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NaiveLineFollower>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
