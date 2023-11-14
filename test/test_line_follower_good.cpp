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
    double calculateDistanceError(const geometry_msgs::msg::TransformStamped& transform)
    {
      return LineFollower::calculateDistanceError(transform);
    } 
    double calculateDistanceEffort(double error)
    {
        return LineFollower::calculateDistanceEffort(error);
    }
    void quaternionToEuler(const tf2::Quaternion &q, double &roll, double &pitch, double &yaw)
    {
        return LineFollower::quaternionToEuler(q, roll, pitch, yaw);
    }
    double calculateYawError(const geometry_msgs::msg::TransformStamped& transform)
    {
      return LineFollower::calculateYawError(transform);
    }    
    double calculateYawEffort(double error)
    {
        return LineFollower::calculateYawEffort(error);
    }
    void publishVelocity(double linear_x, double angular_z){
        return LineFollower::publishVelocity(linear_x, angular_z);
    }
    std::vector<double> GetWaypoints()
    {
        return LineFollower::waypoints;
    }        
};

class TestLineFollowerGood : public ::testing::Test
{
protected:
    void SetUp() override
    {
      rclcpp::init(0, nullptr);
      shim = std::make_shared<LineFollowerShim>();
    }
    void TearDown() override
    {
      rclcpp::shutdown();
    }
    std::shared_ptr<LineFollowerShim> shim;
};

// Test for loadWaypoints function
TEST_F(TestLineFollowerGood, TestloadWaypoints)
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

// Test for calculateDistanceError function
TEST_F(TestLineFollowerGood, TestcalculateDistanceError)
{ 
  // Define transform object 
  geometry_msgs::msg::TransformStamped transformStamped;

  // Define waypoints
  std::vector<double> test_waypoints = {0.00, 0.00, 0.00, 0.00, 5.00, 0.00};
  auto parameter = rclcpp::Parameter("waypoints", test_waypoints);

  // Set and load ROS 2 Params
  shim->set_parameter(parameter);
  shim->loadWaypoints();

  // Calculate distance error
  auto distance_error = shim->calculateDistanceError(transformStamped);
  
  ASSERT_EQ(distance_error, 0.0);
}

// Test for calculateDistanceEffort function
TEST_F(TestLineFollowerGood, TestCalculateDistanceEffort)
{ 
  // Calculate distance effort   
  auto distance_effort = shim->calculateDistanceEffort(0.0); 

  ASSERT_EQ(distance_effort, 0.0);
}

// Test for quaternionToEuler function
TEST_F(TestLineFollowerGood, TestquaternionToEuler)
{  
  // Initialize a quaternion angle
  tf2::Quaternion quaternion(0.0, 0.0, 0.0, 1.0);
  double roll, pitch, yaw;
  
  // Convert the quaternion angle to euler
  shim->quaternionToEuler(quaternion, roll, pitch, yaw);
  
  ASSERT_DOUBLE_EQ(roll, 0.0);
  ASSERT_DOUBLE_EQ(pitch, 0.0);
  ASSERT_DOUBLE_EQ(yaw, 0.0);
}

// Test for calculateYawError function
TEST_F(TestLineFollowerGood, TestcalculateYawError)
{  
  // Define transform object 
  geometry_msgs::msg::TransformStamped transformStamped;

  // Define waypoints
  std::vector<double> test_waypoints = {0.00, 0.00, 0.00, 5.00, 0.00, 0.00};
  auto parameter = rclcpp::Parameter("waypoints", test_waypoints);

  // Set and load ROS 2 Params
  shim->set_parameter(parameter);
  shim->loadWaypoints();

  // Calculate yaw error
  auto yaw_error = shim->calculateYawError(transformStamped);
  std::cout<<"Yaw Error : "<<yaw_error<<std::endl; 
  
  ASSERT_TRUE(yaw_error!=NULL);;
}

// Test for calculateYawEffort function
TEST_F(TestLineFollowerGood, TestcalculateYawEffort)
{  
  // Calculate yaw effort
  auto yaw_effort = shim->calculateYawEffort(0.0); 
  
  ASSERT_EQ(yaw_effort, 0.0);;
}

// Test for PublishVelocity function
TEST_F(TestLineFollowerGood, TestPublishVelocity)
{  
  // Define test ROS 2 Node
  auto test_publish_velocity_node = std::make_shared<rclcpp::Node>("test_publish_velocity_node");

  // Define Twist object
  geometry_msgs::msg::Twist pub_vel_result;

  // Subscribe to the bcr_bot/cmd_vel topic
  auto subscriber = test_publish_velocity_node->create_subscription<geometry_msgs::msg::Twist>("bcr_bot/cmd_vel", 10, 
    [&pub_vel_result](const geometry_msgs::msg::Twist::SharedPtr msg) {
      pub_vel_result = *msg;
    });

  // Publish velocity to bcr_bot/cmd_vel
  shim->publishVelocity(0.05, 0.05);

  // Define executor object
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(test_publish_velocity_node);  

  // Timer callback to stop spinning the nodes
  auto duration = std::chrono::seconds(2); 
  auto timer = test_publish_velocity_node->create_wall_timer(duration, [&executor]() {
      executor.cancel();
  });
  
  executor.spin();
  
  ASSERT_EQ(pub_vel_result.linear.x, 0.05);
  ASSERT_EQ(pub_vel_result.angular.z, 0.05);
}

// Test for ControlLoop function
TEST_F(TestLineFollowerGood, TestControlLoop)
{  
  // Define test ROS 2 Node
  auto test_control_loop_node = std::make_shared<rclcpp::Node>("test_control_loop_node");

  // Define publisher to /tf topic
  auto publisher_ = test_control_loop_node->create_publisher<tf2_msgs::msg::TFMessage>("tf", 10);

  // Boolean to check if a velocity message is received on the bcr_bot/cmd_vel topic
  bool received_twist=false;

  // Subscriber to bcr_bot/cmd_vel
  auto subscriber = test_control_loop_node->create_subscription<geometry_msgs::msg::Twist>("bcr_bot/cmd_vel", 10, 
    [&received_twist](const geometry_msgs::msg::Twist::SharedPtr msg) {
      received_twist=true;
    });

  // Publish a transform message
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "bcr_bot";
  transform.child_frame_id = "empty_world";
  transform.header.stamp = test_control_loop_node->now();

  tf2_msgs::msg::TFMessage tf_message;
  tf_message.transforms.push_back(transform);

  // Define waypoints
  std::vector<double> test_waypoints = {0.00, 0.00, 0.00, 0.00, 5.00, 0.00};
  auto parameter = rclcpp::Parameter("waypoints", test_waypoints);

  // Define executor object and add nodes
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(shim);
  executor.add_node(test_control_loop_node);

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
