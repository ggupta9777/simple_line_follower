#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include "line_follower.hpp"  // The location of your NaiveLineFollower class definition

class TestLineFollower : public ::testing::Test {
protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);
    node = std::make_shared<NaiveLineFollower>();
  }

  void TearDown() override {
    rclcpp::shutdown();
  }

  std::shared_ptr<NaiveLineFollower> node;
};

TEST_F(TestLineFollower, TestCalculateError) {
  // Assuming you have a method in NaiveLineFollower to set waypoints for test purposes
  node->setWaypoints(1.0, 0.0, 5.0, 0.0);  // Set waypoints for a horizontal line (y = 0)

  // Mock a TransformStamped message as it would be returned by tf2
  geometry_msgs::msg::TransformStamped mock_transform;
  mock_transform.transform.translation.x = 2.5;
  mock_transform.transform.translation.y = 2.5;  // Robot is at (2.5, 2.5), away from the line

  // Calculate error
  double error = node->calculateError(mock_transform);

  // For a horizontal line y=0, and robot at (2.5, 2.5), error should be -2.5 (signed distance)
  EXPECT_DOUBLE_EQ(error, -2.5);
}

TEST_F(TestLineFollower, TestCalculateControlEffort) {
  // Mock error
  double mock_error = -2.5;

  // Calculate control effort
  double control_effort = node->calculateControlEffort(mock_error);

  // Assuming a proportional gain (k_p) of 1.0, the control effort should equal the negative error
  EXPECT_DOUBLE_EQ(control_effort, -mock_error);
}

// More tests as needed for other methods...

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}