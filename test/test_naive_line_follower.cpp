#include <memory>
#include <gtest/gtest.h>

#include "simple_line_follower/naive_line_follower.hpp" 

#include <tf2_ros/buffer_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

// Shim class to abstract out NaiveLineFollowerClass
class NaiveLineFollowerShim : public NaiveLineFollower
{ 
  public: 
    NaiveLineFollowerShim() : NaiveLineFollower(){}    
    void loadWaypoints()
    {
      NaiveLineFollower::loadWaypoints();
    } 
};

class TestNaiveLineFollower : public ::testing::Test
{
  protected:
    void SetUp() override
    {
      rclcpp::init(0, nullptr);
      shim_ = std::make_shared<NaiveLineFollowerShim>();

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
    std::shared_ptr<NaiveLineFollowerShim> shim_;
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber_;
    geometry_msgs::msg::TransformStamped transform_;
    tf2_msgs::msg::TFMessage tf_message_;
};


// Test for goToGoal function
TEST_F(TestNaiveLineFollower, TestgoToGoal)
{  
  // Define test ROS 2 Node
  auto test_got_to_goal_node = std::make_shared<rclcpp::Node>("test_go_to_goal_node");
  
  // Define publisher to /tf topic
  publisher_ = test_got_to_goal_node->create_publisher<tf2_msgs::msg::TFMessage>("tf", 1);

  // Boolean to check if a velocity message is received on the bcr_bot/cmd_vel topic
  bool received_twist = false;

  // Subscriber to bcr_bot/cmd_vel
  subscriber_ = test_got_to_goal_node->create_subscription<geometry_msgs::msg::Twist>("bcr_bot/cmd_vel", 1, 
    [&received_twist](const geometry_msgs::msg::Twist::SharedPtr msg) {
      received_twist = true;
  });

  // Define waypoints
  std::vector<double> test_waypoints = {0.00, 0.00, 0.00, 0.00, 5.00, 0.00};
  auto parameter = rclcpp::Parameter("waypoints", test_waypoints);

  // Define executor object and add nodes
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(shim_);
  executor.add_node(test_got_to_goal_node);

  // Set and load ROS 2 Params
  shim_->set_parameter(parameter);
  shim_->loadWaypoints();

  // Publish transform message to /tf topic
  publisher_->publish(tf_message_);

  // Timer callback to stop spinning  
  auto duration = std::chrono::milliseconds(100); 
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
