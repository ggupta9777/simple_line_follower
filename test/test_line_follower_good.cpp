#include <memory>
#include <iomanip>
#include <gtest/gtest.h>

#include "simple_line_follower/line_follower.hpp" 

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/buffer_interface.h>

// Shim class to abstract out LineFollowerClass
class LineFollowerShim : public LineFollower
{ 
  public: 
    LineFollowerShim() : LineFollower(){}    
    void loadWaypoints()
    {
      LineFollower::loadWaypoints();
    }
    void setControlGain()
    {
      LineFollower::setControlGain();
    }
    double calculateDistanceError(const geometry_msgs::msg::TransformStamped& transform)
    {
      return LineFollower::calculateDistanceError(transform);
    }
    double calculateDistanceEffort(double error)
    {
      return LineFollower::calculateDistanceEffort(error);
    }      
};

class TestLineFollowerGood : public ::testing::Test
{
  protected:
    void SetUp() override
    {
      rclcpp::init(0, nullptr);
      shim_ = std::make_shared<LineFollowerShim>();

      transform_.header.frame_id = "base_link";
      transform_.child_frame_id = "odom";
      transform_.transform.translation.x = 0.0;
      transform_.transform.translation.y = 0.0;
      transform_.transform.translation.z = 0.0;

      tf_message_.transforms.push_back(transform_);
    }
    void TearDown() override
    {
      rclcpp::shutdown();
    }
    std::shared_ptr<LineFollowerShim> shim_;
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber_;
    geometry_msgs::msg::TransformStamped transform_;
    tf2_msgs::msg::TFMessage tf_message_;
};


// Testing the calculateDistanceError function where the line is parallel to the X-axis, parallel to the Y-axis and a generic slanted line
TEST_F(TestLineFollowerGood, TestCalculateDistanceError)
{ 
  // Define test ROS 2 Node
  auto test_calculate_distance_error_node = std::make_shared<rclcpp::Node>("test_calculate_distance_error_node");

  // Publisher to /tf
  publisher_ = test_calculate_distance_error_node->create_publisher<tf2_msgs::msg::TFMessage>("tf", 1);

  // Define list of waypoints
  std::vector<std::vector<double>> test_waypoints_list = {
    {0.00, 5.00, 0.00, 5.00, 5.00, 0.00},
    {0.00, 5.00, 0.00, 5.00, 0.00, 0.00},
    {0.00, -5.00, 0.00, 5.00, -5.00, 0.00},
  };

  std::vector<double> expected_outcomes = {5.0, 3.5, -5.0};

  // Define executor object and add nodes
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(shim_);
  executor.add_node(test_calculate_distance_error_node);

  for (int i = 0; i < 3; ++i)
  {
    // Define ROS 2 param which contains the waypoints
    auto parameter = rclcpp::Parameter("waypoints", test_waypoints_list[i]);
    
    // Set and load ROS 2 Params
    shim_->set_parameter(parameter);
    shim_->loadWaypoints();
    
    // Publish transform message to /tf topic
    publisher_->publish(tf_message_);    

    double error = shim_->calculateDistanceError(transform_);

    // Round off to 1 decimal place
    error = std::round(error * 10.0) / 10.0;

    ASSERT_EQ(expected_outcomes[i], error);
  }  
}

// Test for calculateDistanceEffort function
TEST_F(TestLineFollowerGood, TestCalculateDistanceEffort)
{ 
  double test_k_p = 1.5;
  auto parameter = rclcpp::Parameter("k_p", test_k_p);

  // Set and load ROS 2 Params
  shim_->set_parameter(parameter);
  shim_->setControlGain();

  // Calculate distance effort   
  auto distance_effort = shim_->calculateDistanceEffort(1.5); 
  ASSERT_EQ(distance_effort, 2.25);
}

// Test for PublishVelocity function
TEST_F(TestLineFollowerGood, TestPublishVelocity)
{  
  // Define test ROS 2 Node
  auto test_publish_velocity_node = std::make_shared<rclcpp::Node>("test_publish_velocity_node");

  // Define Twist object
  geometry_msgs::msg::Twist pub_vel_result;

  // Boolean to check if any message has been received on /bcr_bot/cmd_vel
  bool received_twist = false;

  // Subscribe to the bcr_bot/cmd_vel topic
  subscriber_ = test_publish_velocity_node->create_subscription<geometry_msgs::msg::Twist>("bcr_bot/cmd_vel", 1, 
    [&pub_vel_result, &received_twist](const geometry_msgs::msg::Twist::SharedPtr msg) {
      pub_vel_result = *msg;
      received_twist = true;
    });

  // Publish velocity to bcr_bot/cmd_vel
  shim_->publishVelocity(0.05, 0.05);

  // Define executor object
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(test_publish_velocity_node);  

  // Timer callback to stop spinning the nodes
  auto duration = std::chrono::seconds(2); 
  auto timer = test_publish_velocity_node->create_wall_timer(duration, [&executor]() {
      executor.cancel();
  });
  
  executor.spin();
  
  ASSERT_TRUE(received_twist);
  ASSERT_EQ(pub_vel_result.linear.x, 0.05);
  ASSERT_EQ(pub_vel_result.angular.z, 0.05);
}

// Test for ControlLoop function
TEST_F(TestLineFollowerGood, TestControlLoop)
{  
  // Initialize proportional gain
  double test_k_p = 2.5;
  auto k_p_parameter = rclcpp::Parameter("k_p", test_k_p);

  // Define test ROS 2 Node
  auto test_control_loop_node = std::make_shared<rclcpp::Node>("test_control_loop_node");

  // Define publisher to /tf topic
  publisher_ = test_control_loop_node->create_publisher<tf2_msgs::msg::TFMessage>("tf", 1);

  // Define Twist object
  geometry_msgs::msg::Twist pub_vel_result;

  // Subscribe to the bcr_bot/cmd_vel topic
  subscriber_ = test_control_loop_node->create_subscription<geometry_msgs::msg::Twist>("bcr_bot/cmd_vel", 1, 
    [&pub_vel_result](const geometry_msgs::msg::Twist::SharedPtr msg) {
      pub_vel_result = *msg;
  });

  // Define waypoints
  std::vector<double> test_waypoints = {0.00, 1.00, 0.00, 5.00, 1.00, 0.00};
  auto waypoint_parameter = rclcpp::Parameter("waypoints", test_waypoints);

  // Define executor object and add nodes
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(shim_);
  executor.add_node(test_control_loop_node);

  // Set and load ROS 2 Params
  shim_->set_parameter(k_p_parameter);
  shim_->set_parameter(waypoint_parameter);
  shim_->loadWaypoints();
  shim_->setControlGain();

  // Publish transform message to /tf topic
  publisher_->publish(tf_message_);
  
  // Timer callback to stop spinning
  auto duration = std::chrono::milliseconds(50); 
  auto timer = shim_->create_wall_timer(duration, [&executor]() {
    executor.cancel();
  });
  
  executor.spin();

  ASSERT_EQ(pub_vel_result.angular.z , 2.5);
}

int main(int argc, char **argv) 
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
