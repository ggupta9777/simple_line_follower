#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.h>
#include <tf2_msgs/msg/tf_message.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

class TfBroadcaster : public rclcpp::Node
{
public:
    TfBroadcaster() : Node("TfBroadcaster")
    {
        // Initialize the transform broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        tf_broadcaster_static_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);

        pose_subscriber = this->create_subscription<tf2_msgs::msg::TFMessage>(
            "/model/bcr_bot/pose", 1,
            std::bind(&TfBroadcaster::PoseCallback, this, std::placeholders::_1)
        );

        pose_subscriber_static_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
            "/model/bcr_bot/pose_static", 1,
            std::bind(&TfBroadcaster::PoseStaticCallback, this, std::placeholders::_1)
        );
    }

private:
    void PoseCallback(tf2_msgs::msg::TFMessage::SharedPtr msg)
    {
        tf_broadcaster_->sendTransform(msg->transforms);
    }

    void PoseStaticCallback(tf2_msgs::msg::TFMessage::SharedPtr msg)
    {
        tf_broadcaster_static_->sendTransform(msg->transforms);
    }

    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr pose_subscriber;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr pose_subscriber_static_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_static_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TfBroadcaster>());
    rclcpp::shutdown();
    return 0;
}
