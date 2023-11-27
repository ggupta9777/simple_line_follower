#include <gtest/gtest.h>

#include "simple_line_follower/naive_line_follower.hpp" 

// Shim class to abstract out NaiveLineFollowerClass
class NaiveLineFollowerShim : public NaiveLineFollower
{ 
  public: 
    NaiveLineFollowerShim() : NaiveLineFollower() {}    
    void loadWaypoints()
    {
      NaiveLineFollower::loadWaypoints();
    }
    void updateBotPosition(double new_bot_x, double new_bot_y)
    {
      bot_x = new_bot_x;
      bot_y = new_bot_y;
    } 
};

class TestNaiveLineFollower : public ::testing::Test
{
  protected:
    void SetUp() override
    {
      rclcpp::init(0, nullptr);
      shim_ = std::make_shared<NaiveLineFollowerShim>();
    }
    void TearDown() override
    {
      rclcpp::shutdown();
    }
    std::shared_ptr<NaiveLineFollowerShim> shim_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber_;
};


// Test for goToGoal function
TEST_F(TestNaiveLineFollower, TestgoToGoal)
{  
  // Define test ROS 2 Node
  auto test_got_to_goal_node = std::make_shared<rclcpp::Node>("test_go_to_goal_node");

  // Boolean to check if a velocity message is received on the bcr_bot/cmd_vel topic
  bool received_twist = false;

  // Subscriber to bcr_bot/cmd_vel
  subscriber_ = test_got_to_goal_node->create_subscription<geometry_msgs::msg::Twist>("bcr_bot/cmd_vel", 1, 
    [&received_twist](const geometry_msgs::msg::Twist::SharedPtr) {
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

  // Set position of bot
  shim_->updateBotPosition(0.0, 5.0);

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
