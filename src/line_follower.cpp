#include "simple_line_follower/line_follower.hpp"

LineFollower::LineFollower()
: Node("line_follower"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
{
  // Initialize publisher
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("bcr_bot/cmd_vel", 10);

  // Declare ROS 2 Param to get waypoints
  this->declare_parameter("waypoints", rclcpp::PARAMETER_DOUBLE_ARRAY) ;

  loadWaypoints();

  // Create a timer to call the controlLoop
  control_loop_timer_ = this->create_wall_timer(  
    std::chrono::milliseconds(100),
    std::bind(&LineFollower::controlLoop, this));
} 

// Function to Load the ROS 2 Param to get waypoints
void LineFollower::loadWaypoints() {
  try
  {
    auto waypoints_param = this->get_parameter("waypoints") ;
  
    // Converting waypoints_param to a vector<double>
    waypoints = waypoints_param.as_double_array();

    waypoint_1_.x = waypoints[0]; waypoint_1_.y = waypoints[1]; waypoint_1_.z = waypoints[2];
    waypoint_2_.x = waypoints[3]; waypoint_2_.y = waypoints[4]; waypoint_2_.z = waypoints[5];
    waypoint_angle = std::atan2(waypoint_2_.y  - waypoint_1_.y , waypoint_2_.x  - waypoint_1_.x );

    RCLCPP_INFO(this->get_logger(), "Waypoints set successfully");

  }catch (const std::exception& e){        
    RCLCPP_INFO(this->get_logger(), "Exception caught: %s", e.what());
  }    
}

// Function to calculate error while following the line
double LineFollower::calculateDistanceError(const geometry_msgs::msg::TransformStamped& transform)
{
  double goal_x = waypoint_2_.x - waypoint_1_.x;
  double goal_y = waypoint_2_.y - waypoint_1_.y;
  
  // Distance between a point and a line
  double error = (goal_y * transform.transform.translation.x -
    goal_x * transform.transform.translation.y +
    waypoint_2_.x * waypoint_1_.y -
    waypoint_2_.y * waypoint_1_.x) /
    std::sqrt(std::pow(goal_y, 2) + std::pow(goal_x, 2));
    
  return error;
}

// Function to calculate the control effort from the error.
double LineFollower::calculateDistanceEffort(double error)
{
  double k_p = 0.1; // Proportional gain
  return k_p * error;
}

// Function to convert Quaternion to Euler
void LineFollower::quaternionToEuler(const tf2::Quaternion &q, double &roll, double &pitch, double &yaw)
{
  // Function to change Quaternions to Eulers  
  tf2::Matrix3x3 m(q);
  m.getRPY(roll, pitch, yaw);
}

// Function to calculate angular error between bot's yaw and angle formed by the line
double LineFollower::calculateYawError(const geometry_msgs::msg::TransformStamped& transform)
{
  tf2::Quaternion q(transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w);
  quaternionToEuler(q, bot_roll, bot_pitch, bot_yaw)  ;

  // Convert the bot yaw angle from range[-pi, pi] to [0, 2*pi]
  if (bot_yaw<=0){
    bot_yaw = 2*M_PI + bot_yaw ;
  }
  return (waypoint_angle-bot_yaw);
}

// Function to calculate the yaw effort from the yaw error.
double LineFollower::calculateYawEffort(double error)
{
  double k_p = 0.5; // Proportional gain
  return k_p * error;
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
  geometry_msgs::msg::TransformStamped transformStamped;

  try{
    // Lookup transform to get the bot's position
    transformStamped = tf_buffer_.lookupTransform("empty_world", "bcr_bot", tf2::TimePointZero);
    
    if(!yaw_fixed){

      // Calculate angular error between bot's yaw and angle formed by the line
      yaw_error = calculateYawError(transformStamped);

      // Calculate the effort required to minimize the yaw_error
      double yaw_effort = calculateYawEffort(yaw_error);

      // Publish Velocity messages
      publishVelocity(0.0, yaw_effort);
      
      if (std::abs(yaw_error) < 0.05){
        yaw_fixed = true;
      }
      RCLCPP_INFO(this->get_logger(), "Fixing Yaw");
    }
    else{

      // Calculate distance between bot and the line
      double distance_error = calculateDistanceError(transformStamped);

      // Calculate the effort required to minimize the distance_error 
      double control_effort = calculateDistanceEffort(distance_error);

      // Publish Velocity messages
      publishVelocity(1.0, control_effort);
      RCLCPP_INFO(this->get_logger(), "Following Line");
    }        
    // constant forward velocity and dynamic angular velocity
    }catch (const tf2::TransformException &ex) {
    RCLCPP_ERROR(this->get_logger(), "Could not transform %s to %s: %s", "empty_world", "bcr_bot", ex.what());
    }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LineFollower>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
