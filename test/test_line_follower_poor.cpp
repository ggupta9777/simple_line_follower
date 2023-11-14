#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "simple_line_follower/naive_line_follower.hpp" 
#include <tf2_ros/buffer_interface.h>
#include <memory>

using std::placeholders::_1;

// Shim class to abstract out NaiveLineFollowerClass
class NaiveLineFollowerShim : public NaiveLineFollower
{ 
  public: 
    NaiveLineFollowerShim() : NaiveLineFollower(){}    
    void loadWaypoints()
    {
        NaiveLineFollower::loadWaypoints();
    }
    std::vector<double> GetWaypoints()
    {
        return NaiveLineFollower::waypoints;
    }   
};

class TestNaiveLineFollower : public ::testing::Test
{
protected:
    void SetUp() override
    {
      rclcpp::init(0, nullptr);
      shim = std::make_shared<NaiveLineFollowerShim>();
    }
    void TearDown() override
    {
      rclcpp::shutdown();
    }
    std::shared_ptr<NaiveLineFollowerShim> shim;
};

// Test for loadWaypoints function
TEST_F(TestNaiveLineFollower, TestloadWaypoints)
{  
  // Define waypoints
  std::vector<double> test_waypoints = {0.00, 0.00, 0.00, 0.00, 5.00, 0.00};
  auto parameter = rclcpp::Parameter("waypoints", test_waypoints);

  // Initialize executor object
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(shim);

  // Set and load ROS 2 Params
  shim->set_parameter(parameter);
  shim->loadWaypoints();

  // Timer callback to stop spinning    
  auto duration = std::chrono::milliseconds(100); 
  auto timer = shim->create_wall_timer(duration, [&executor]() {
    executor.cancel();
  });
  
  executor.spin();

  ASSERT_EQ(test_waypoints, shim->GetWaypoints());
}

// Test for goToGoal function
TEST_F(TestNaiveLineFollower, TestgoToGoal)
{  
  // Define test ROS 2 Node
  auto test_got_to_goal_node = std::make_shared<rclcpp::Node>("test_go_to_goal_node");
  
  // Define publisher to /tf topic
  auto publisher_ = test_got_to_goal_node->create_publisher<tf2_msgs::msg::TFMessage>("tf", 10);

  // Boolean to check if a velocity message is received on the bcr_bot/cmd_vel topic
  bool received_twist=false;

  // Subscriber to bcr_bot/cmd_vel
  auto subscriber = test_got_to_goal_node->create_subscription<geometry_msgs::msg::Twist>("bcr_bot/cmd_vel", 10, 
    [&received_twist](const geometry_msgs::msg::Twist::SharedPtr msg) {
      received_twist=true;
    });

  // Publish a transform message
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "bcr_bot";
  transform.child_frame_id = "empty_world";
  transform.header.stamp =test_got_to_goal_node->now();

  tf2_msgs::msg::TFMessage tf_message;
  tf_message.transforms.push_back(transform);

  // Define waypoints
  std::vector<double> test_waypoints = {0.00, 0.00, 0.00, 0.00, 5.00, 0.00};
  auto parameter = rclcpp::Parameter("waypoints", test_waypoints);

  // Define executor object and add nodes
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(shim);
  executor.add_node(test_got_to_goal_node);

  // Set and load ROS 2 Params
  shim->set_parameter(parameter);
  shim->loadWaypoints();

  // Publish transform message to /tf topic
  publisher_->publish(tf_message);

  // Timer callback to stop spinning  
  auto duration = std::chrono::milliseconds(200); 
  auto timer = shim->create_wall_timer(duration, [&executor]() {
    executor.cancel();
  });

  executor.spin();

  ASSERT_TRUE(received_twist);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
