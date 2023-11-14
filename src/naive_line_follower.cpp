#include "simple_line_follower/naive_line_follower.hpp"

NaiveLineFollower::NaiveLineFollower()
: Node("naive_line_follower"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
{
  // Initialize publisher
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("bcr_bot/cmd_vel", 10);

  // Declare ROS 2 Param to get waypoints
  this->declare_parameter("waypoints", rclcpp::PARAMETER_DOUBLE_ARRAY) ;

  loadWaypoints();

  // Create a timer to call goToGoal
  control_loop_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&NaiveLineFollower::goToGoal, this));
}

// Function to Load the ROS 2 Param to get waypoints
void NaiveLineFollower::loadWaypoints() {
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

void NaiveLineFollower::goToGoal() {

  geometry_msgs::msg::TransformStamped transformStamped;
  
  try {
    // Lookup transform to get the bot's position
    transformStamped = tf_buffer_.lookupTransform("empty_world", "bcr_bot", tf2::TimePointZero, std::chrono::milliseconds(100));

    if(!yaw_fixed){

      // Convert Quaternion to Euler
      tf2::Quaternion q(transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z, transformStamped.transform.rotation.w);
      tf2::Matrix3x3 m(q);
      m.getRPY(bot_roll, bot_pitch, bot_yaw);

      // Convert the bot yaw angle from range[-pi, pi] to [0, 2*pi]
      if (bot_yaw<=0){
          bot_yaw = 2*M_PI + bot_yaw ;
      }
      
      // Angular error between bot's yaw and angle formed by the line
      yaw_error = waypoint_angle-bot_yaw;
      
      // Apply proportional control
      double k_p = 0.5; 
      
      // Calculate the yaw effort from the yaw error
      yaw_effort = k_p * yaw_error;
      
      // Create and publish the Twist message
      geometry_msgs::msg::Twist cmd_vel;
      cmd_vel.linear.x = 0;
      cmd_vel.angular.z = yaw_effort;
      cmd_vel_pub_->publish(cmd_vel);
      if (std::abs(yaw_error) < 0.05){
        yaw_fixed = true;
      }
      RCLCPP_INFO(this->get_logger(), "Fixing Yaw");        
    }      
    else{
      // Calculate error while following the line
      double goal_x = waypoint_2_.x - waypoint_1_.x;
      double goal_y = waypoint_2_.y - waypoint_1_.y;
      
      // Distance between a point and a line
      distance_error = (goal_y * transformStamped.transform.translation.x -
        goal_x * transformStamped.transform.translation.y +
        waypoint_2_.x * waypoint_1_.y -
        waypoint_2_.y * waypoint_1_.x) /
        std::sqrt(std::pow(goal_y, 2) + std::pow(goal_x, 2));

      // Apply proportional control
      double k_p = 0.1;

      // Calculate the distance effort from the distance error
      distance_effort = k_p * distance_error;
      
      // Create and publish the Twist message
      geometry_msgs::msg::Twist cmd_vel;
      cmd_vel_.linear.x = 1.0; // constant forward velocity
      cmd_vel_.angular.z = distance_effort; // turning rate
      cmd_vel_pub_->publish(cmd_vel_);
      
      RCLCPP_INFO(this->get_logger(), "Following Line");
    }
    
  }catch (const tf2::TransformException &ex) {
  RCLCPP_ERROR(this->get_logger(), "Could not transform %s to %s: %s", "empty_world", "bcr_bot", ex.what());
  }
}
  
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NaiveLineFollower>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
