#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "simple_line_follower/line_follower.hpp" 
#include <tf2_ros/buffer_interface.h>
#include <memory>

using std::placeholders::_1;

// Shim class to abstract out LineFollowerClass
class LineFollowerShim : public LineFollower
{ 
  public: 
    LineFollowerShim() : LineFollower(){}    
    void loadWaypoints()
    {
        LineFollower::loadWaypoints();
    }
    std::vector<double> GetWaypoints()
    {
        return LineFollower::waypoints;
    }        
};

class TestLineFollower : public ::testing::Test
{
protected:
    void SetUp() override
    {
      rclcpp::init(0, nullptr);
      shim = std::make_shared<LineFollowerShim>();

      transform.header.frame_id = "bcr_bot";
      transform.child_frame_id = "empty_world";
      transform.transform.translation.x = 0.0;
      transform.transform.translation.y = 0.0;
      transform.transform.translation.z = 0.0;

      tf_message.transforms.push_back(transform);
    }
    void TearDown() override
    {
      rclcpp::shutdown();
    }
    std::shared_ptr<LineFollowerShim> shim;
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber_;
    geometry_msgs::msg::TransformStamped transform;
    tf2_msgs::msg::TFMessage tf_message;
};

// Testing for identical waypoints to check for division by zero condition
TEST_F(TestLineFollower, TestIdenticalWaypoints)
{  
  // Define test ROS 2 Node
  auto test_identical_waypoints_node = std::make_shared<rclcpp::Node>("test_identical_waypoints_node");
  
  // Publisher to /tf
  publisher_ = test_identical_waypoints_node->create_publisher<tf2_msgs::msg::TFMessage>("tf", 10);

  // Boolean to check if a velocity message is received on the bcr_bot/cmd_vel topic
  bool received_twist=false;

  // Subscriber to bcr_bot/cmd_vel
  subscriber_ = test_identical_waypoints_node->create_subscription<geometry_msgs::msg::Twist>("bcr_bot/cmd_vel", 10, 
        [&received_twist](const geometry_msgs::msg::Twist::SharedPtr msg) {
          received_twist=true;
        });

  // Define waypoints
  std::vector<double> test_waypoints = {0.00, 0.00, 0.00, 0.00, 0.00, 0.00};
  auto parameter = rclcpp::Parameter("waypoints", test_waypoints);

  // Define executor object and add nodes
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(shim);
  executor.add_node(test_identical_waypoints_node);

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

  ASSERT_EQ(test_waypoints, shim->GetWaypoints());
  ASSERT_TRUE(received_twist);
}

// Testing for waypoints in which the line is anti-parallel to the bot pose
TEST_F(TestLineFollower, TestAntiParallelWaypoints)
{ 
  // Define test ROS 2 Node
  auto test_antiparallel_waypoints_node = std::make_shared<rclcpp::Node>("test_antiparallel_waypoints_node");

  // Publisher to /tf
  publisher_ = test_antiparallel_waypoints_node->create_publisher<tf2_msgs::msg::TFMessage>("tf", 10);

  // Boolean to check if a velocity message is received on the bcr_bot/cmd_vel topic
  bool received_twist=false;

  // Subscriber to bcr_bot/cmd_vel
  subscriber_ = test_antiparallel_waypoints_node->create_subscription<geometry_msgs::msg::Twist>("bcr_bot/cmd_vel", 10, 
        [&received_twist](const geometry_msgs::msg::Twist::SharedPtr msg) {
          received_twist=true;
        });

  // Define waypoints
  std::vector<double> test_waypoints = {0.00, 0.00, 0.00, 0.00, -5.00, 0.00};
  auto parameter = rclcpp::Parameter("waypoints", test_waypoints);

  // Define executor object and add nodes
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(shim);
  executor.add_node(test_antiparallel_waypoints_node);

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

  ASSERT_EQ(test_waypoints, shim->GetWaypoints());
  ASSERT_TRUE(received_twist);
}

// Testing for waypoints in which the line is perpendicular to the bot pose
TEST_F(TestLineFollower, TestPerpendicularWaypoints)
{ 
  // Define test ROS 2 Node
  auto test_perpendicular_waypoints_node = std::make_shared<rclcpp::Node>("test_perpendicular_waypoints_node");

  // Publisher to /tf
  publisher_ = test_perpendicular_waypoints_node->create_publisher<tf2_msgs::msg::TFMessage>("tf", 10);
  
  // Boolean to check if a velocity message is received on the bcr_bot/cmd_vel topic
  bool received_twist=false; 

  // Subscriber to bcr_bot/cmd_vel
  subscriber_ = test_perpendicular_waypoints_node->create_subscription<geometry_msgs::msg::Twist>("bcr_bot/cmd_vel", 10, 
        [&received_twist](const geometry_msgs::msg::Twist::SharedPtr msg) {
          received_twist=true;
        });

  // Define waypoints
  std::vector<double> test_waypoints = {0.00, 0.00, 0.00, 5.00, 0.00, 0.00};
  auto parameter = rclcpp::Parameter("waypoints", test_waypoints);

  // Define executor object and add nodes
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(shim);
  executor.add_node(test_perpendicular_waypoints_node);

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

  ASSERT_EQ(test_waypoints, shim->GetWaypoints());
  ASSERT_TRUE(received_twist);
}

// Testing for waypoints in which the line is parallel to the X-axis, parallel to the Y-axis and a generic slanted line
TEST_F(TestLineFollower, TestMultipleWaypoints)
{ 
  // Define test ROS 2 Node
  auto test_multiple_waypoints_node = std::make_shared<rclcpp::Node>("test_multiple_waypoints_node");

  // Publisher to /tf
  publisher_ = test_multiple_waypoints_node->create_publisher<tf2_msgs::msg::TFMessage>("tf", 10);

  // Boolean to check if a velocity message is received on the bcr_bot/cmd_vel topic
  bool received_twist=false; 

  // Subscriber to bcr_bot/cmd_vel
  subscriber_ = test_multiple_waypoints_node->create_subscription<geometry_msgs::msg::Twist>("bcr_bot/cmd_vel", 10, 
        [&received_twist](const geometry_msgs::msg::Twist::SharedPtr msg) {
          received_twist=true;
        });

  // Define list of waypoints
  std::vector<std::vector<double>> test_waypoints_list = {
    {0.00, 0.00, 0.00, 5.00, 0.00, 0.00},
    {0.00, 0.00, 0.00, 5.00, 5.00, 0.00},
    {0.00, 0.00, 0.00, 0.00, -5.00, 0.00},
  };

  // Define executor object and add nodes
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(shim);
  executor.add_node(test_multiple_waypoints_node);

  for (const auto& test_waypoints : test_waypoints_list){
        // Boolean to check if a velocity message is received on the bcr_bot/cmd_vel topic
        received_twist=false;

        // Define ROS 2 param which contains the waypoints
        auto parameter = rclcpp::Parameter("waypoints", test_waypoints);

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

        ASSERT_EQ(test_waypoints, shim->GetWaypoints());
        ASSERT_TRUE(received_twist);
    }  
}

int main(int argc, char **argv) 
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
