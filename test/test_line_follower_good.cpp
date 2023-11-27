#include <gtest/gtest.h>

#include "simple_line_follower/line_follower.hpp" 

// Shim class to abstract out LineFollowerClass
class LineFollowerShim : public LineFollower
{ 
  public: 
    LineFollowerShim() : LineFollower() {}    
    void loadWaypoints()
    {
      LineFollower::loadWaypoints();
    }
    void setControlGain()
    {
      LineFollower::setControlGain();
    }
    double calculateDistanceError()
    {
      return LineFollower::calculateDistanceError();
    }
    double calculateProportionalEffort(double error)
    {
      return LineFollower::calculateProportionalEffort(error);
    }
    double calculateDerivativeEffort(double error)
    {
      return LineFollower::calculateDerivativeEffort(error);
    }
    void updateBotPosition(double new_bot_x, double new_bot_y)
    {
      bot_x = new_bot_x;
      bot_y = new_bot_y;
    }      
};

class TestLineFollowerGood : public ::testing::Test
{
  protected:
    void SetUp() override
    {
      rclcpp::init(0, nullptr);
      shim_ = std::make_shared<LineFollowerShim>();
    }
    void TearDown() override
    {
      rclcpp::shutdown();
    }
    std::shared_ptr<LineFollowerShim> shim_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber_;
};


// Testing the calculateDistanceError function
TEST_F(TestLineFollowerGood, TestCalculateDistanceError)
{ 
  // Define list of waypoints
  std::vector<std::vector<double>> test_waypoints_list = {
    {0.00, 5.00, 0.00, 5.00, 5.00, 0.00},
    {0.00, 5.00, 0.00, 5.00, 0.00, 0.00},
    {0.00, -5.00, 0.00, 5.00, -5.00, 0.00},
  };

  std::vector<double> expected_outcomes = {8.0, 2.1, -2.0};

  for (int i = 0; i < 3; ++i)
  {
    // Define ROS 2 param which contains the waypoints
    auto parameter = rclcpp::Parameter("waypoints", test_waypoints_list[i]);
    
    // Set and load ROS 2 Params
    shim_->set_parameter(parameter);
    shim_->loadWaypoints();
    
    // Set position of bot
    shim_->updateBotPosition(5.0, -3.0);   

    double error = shim_->calculateDistanceError();

    // Round off to 1 decimal place
    error = std::round(error * 10.0) / 10.0;

    ASSERT_EQ(expected_outcomes[i], error);
  }  
}

// Test for calculateProportionalEffort function
TEST_F(TestLineFollowerGood, TestCalculateProportionalEffort)
{ 
  double test_k_p = 1.5;
  auto parameter_k_p = rclcpp::Parameter("k_p", test_k_p);

  // Set and load ROS 2 Params
  shim_->set_parameter(parameter_k_p);
  shim_->setControlGain();

  // Calculate proportional effort   
  auto prop_effort = shim_->calculateProportionalEffort(1.5); 
  ASSERT_EQ(prop_effort, 2.25);
}

// Test for calculateDerivativeEffort function
TEST_F(TestLineFollowerGood, TestCalculateDerivativeEffort)
{ 
  double test_k_d = 0.6;
  auto parameter_k_d = rclcpp::Parameter("k_d", test_k_d);

  // Set and load ROS 2 Params
  shim_->set_parameter(parameter_k_d);
  shim_->setControlGain();

  // Calculate distance effort   
  auto der_effort = shim_->calculateDerivativeEffort(2.0); 
  ASSERT_EQ(der_effort, 1.20);
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

  // Initialize derivative gain
  double test_k_d = 2.0;
  auto k_d_parameter = rclcpp::Parameter("k_d", test_k_d);

  // Define test ROS 2 Node
  auto test_control_loop_node = std::make_shared<rclcpp::Node>("test_control_loop_node");

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
  shim_->set_parameter(k_d_parameter);
  shim_->set_parameter(waypoint_parameter);
  shim_->loadWaypoints();
  shim_->setControlGain();

  // Set position of bot
  shim_->updateBotPosition(0.0, 0.0);
  
  // Timer callback to stop spinning
  auto duration = std::chrono::milliseconds(50); 
  auto timer = shim_->create_wall_timer(duration, [&executor]() {
    executor.cancel();
  });
  
  executor.spin();

  ASSERT_EQ(pub_vel_result.angular.z , 4.5);
}

int main(int argc, char **argv) 
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
