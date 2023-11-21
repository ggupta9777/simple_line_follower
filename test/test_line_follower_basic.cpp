#include <memory>
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
};

class TestLineFollowerBasic : public ::testing::Test
{
  protected:
    void SetUp() override
    {
      rclcpp::init(0, nullptr);
      shim_ = std::make_shared<LineFollowerShim>();

      // Make transform message
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

// Test the control loop
TEST_F(TestLineFollowerBasic, TestControlLoop)
{ 
  // Define test ROS 2 Node
  auto test_control_loop_node = std::make_shared<rclcpp::Node>("test_control_loop_node");

  // Publisher to /tf
  publisher_ = test_control_loop_node->create_publisher<tf2_msgs::msg::TFMessage>("tf", 1);
  
  // Boolean to check if a velocity message is received on the bcr_bot/cmd_vel topic
  bool received_twist = false; 

  // Subscriber to bcr_bot/cmd_vel
  subscriber_ = test_control_loop_node->create_subscription<geometry_msgs::msg::Twist>("bcr_bot/cmd_vel", 1, 
    [&received_twist](const geometry_msgs::msg::Twist::SharedPtr msg) {
      received_twist = true;
  });

  // Define waypoints
  std::vector<double> test_waypoints = {0.00, 0.00, 0.00, 5.00, 0.00, 0.00};
  auto parameter = rclcpp::Parameter("waypoints", test_waypoints);

  // Define executor object and add nodes
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(shim_);
  executor.add_node(test_control_loop_node);

  // Set and load ROS 2 Params
  shim_->set_parameter(parameter);
  shim_->loadWaypoints();

  // Publish transform message to /tf topic
  publisher_->publish(tf_message_);
  
  // Timer callback to stop spinning
  auto duration = std::chrono::milliseconds(200); 
  auto timer = shim_->create_wall_timer(duration, [&executor]() {
    executor.cancel();
  });
    
  executor.spin();

  ASSERT_TRUE(received_twist);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
