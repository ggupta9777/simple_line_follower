#include <gtest/gtest.h>
#include <memory>
#include "naive_line_follower.hpp"
#include "geometry_msgs/msg/point.hpp"

class NaiveLineFollowerTest : public ::testing::Test {
protected:
  std::shared_ptr<NaiveLineFollower> node;

  void SetUp() override {
    node = std::make_shared<NaiveLineFollower>();
  }
};

TEST_F(NaiveLineFollowerTest, LoadWaypointsTest) {
  geometry_msgs::msg::Point wp1 = node->getWaypoint1();
  geometry_msgs::msg::Point wp2 = node->getWaypoint2();

  EXPECT_DOUBLE_EQ(wp1.x, 1.0);
  EXPECT_DOUBLE_EQ(wp1.y, 0.0);
  EXPECT_DOUBLE_EQ(wp1.z, 0.0);

  EXPECT_DOUBLE_EQ(wp2.x, 5.0);
  EXPECT_DOUBLE_EQ(wp2.y, 0.0);
  EXPECT_DOUBLE_EQ(wp2.z, 0.0);
}

TEST_F(NaiveLineFollowerTest, ControlErrorHorizontalLine) {
  geometry_msgs::msg::TransformStamped fakeTransform;
  fakeTransform.transform.translation.x = 3.0;  // Robot is halfway between the waypoints
  fakeTransform.transform.translation.y = 0.0;

  double error = node->calculateError(fakeTransform);
  EXPECT_DOUBLE_EQ(error, 0.0);  // On a horizontal line, the error should be zero if the robot is exactly on the line
}

TEST_F(NaiveLineFollowerTest, ControlErrorVerticalLine) {
  node->setWaypoints(0.0, 1.0, 0.0, 0.0, 5.0, 0.0);  // Setting waypoints for a vertical line

  geometry_msgs::msg::TransformStamped fakeTransform;
  fakeTransform.transform.translation.x = 0.0;
  fakeTransform.transform.translation.y = 3.0;  // Robot is halfway between the waypoints

  double error = node->calculateError(fakeTransform);
  EXPECT_DOUBLE_EQ(error, 0.0);  // On a vertical line, the error should be zero if the robot is exactly on the line
}

TEST_F(NaiveLineFollowerTest, ControlErrorSlantedLine) {
  node->setWaypoints(0.0, 0.0, 0.0, 1.0, 1.0, 0.0);  // Setting waypoints for a slanted line (45 degrees)

  geometry_msgs::msg::TransformStamped fakeTransform;
  fakeTransform.transform.translation.x = 0.5;
  fakeTransform.transform.translation.y = 0.5;  // Robot is on the line

  double error = node->calculateError(fakeTransform);
  EXPECT_NEAR(error, 0.0, 1e-6);  // The error should be nearly zero if the robot is exactly on the slanted line
}

TEST_F(NaiveLineFollowerTest, ControlEffortZeroError) {
  double control_effort = node->calculateControlEffort(0.0);
  EXPECT_DOUBLE_EQ(control_effort, 0.0);
}

TEST_F(NaiveLineFollowerTest, ControlEffortOneMeterError) {
  double control_effort = node->calculateControlEffort(1.0);
  // Assuming a proportional gain (k_p) of 1.0, the control effort should equal the error
  EXPECT_DOUBLE_EQ(control_effort, 1.0);
}

TEST_F(NaiveLineFollowerTest, ControlEffortTenMeterError) {
  double control_effort = node->calculateControlEffort(10.0);
  // Assuming a proportional gain (k_p) of 1.0, the control effort should equal the error
  EXPECT_DOUBLE_EQ(control_effort, 10.0);
}

// Mock or simulate the TF listener behavior in the ControlLoopTest
// This may involve creating a fake TF listener or using a test-specific subclass of NaiveLineFollower that overrides the TF listener behavior

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
