#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include "naive_line_follower.hpp"  // The location of your NaiveLineFollower class definition
#include <tf2_ros/buffer_interface.h>
#include <memory>

using std::placeholders::_1;

class TestNaiveLineFollower : public ::testing::Test {
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

// Mock TransformListener to return a fixed transform
class MockTransformListener : public tf2_ros::TransformListener {
public:
  MockTransformListener(tf2_ros::BufferInterface& buffer)
    : TransformListener(buffer, true) {}

  void setTransform(const geometry_msgs::msg::TransformStamped& transform) {
    transform_ = transform;
  }

protected:
  void onInit() override {
    // Override to prevent initialization of the actual TransformListener
  }

  tf2::TransformableRequestHandle_t addTransformableRequest(
    tf2::TransformableCallback callback,
    const std::string& target_frame,
    const std::string& source_frame,
    tf2::TimePoint time,
    tf2::Duration timeout) override {
      // Immediately invoke the callback with the mock transform
      callback(transform_);
      return {};
  }
};

TEST_F(TestNaiveLineFollower, TestControlLoop) {
  // Set a fixed transform for the test
  geometry_msgs::msg::TransformStamped mock_transform;
  mock_transform.header.frame_id = "odom";
  mock_transform.child_frame_id = "base_link";
  mock_transform.transform.translation.x = 2.5;
  mock_transform.transform.translation.y = 2.5;
  mock_transform.transform.rotation.w = 1.0;

  // Use the MockTransformListener to provide the fixed transform
  MockTransformListener mock_tf_listener(node->get_tf_buffer());
  mock_tf_listener.setTransform(mock_transform);

  // Run the controlLoop method
  node->controlLoop();

  // Check if cmd_vel was published correctly
  auto cmd_vel = node->get_last_cmd_vel();
  // Assuming a method get_last_cmd_vel() is implemented to retrieve the last cmd_vel
  EXPECT_NEAR(cmd_vel.linear.x, 0.5, 1e-2);
  EXPECT_NEAR(cmd_vel.angular.z, /* expected value */, 1e-2); // You need to calculate the expected value
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
