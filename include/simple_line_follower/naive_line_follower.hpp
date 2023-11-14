#ifndef NAIVE_LINE_FOLLOWER_HPP
#define NAIVE_LINE_FOLLOWER_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "nav_msgs/msg/odometry.hpp"

class NaiveLineFollower : public rclcpp::Node {
    public :
        NaiveLineFollower();
        void loadWaypoints();
        void goToGoal();

        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
        rclcpp::TimerBase::SharedPtr control_loop_timer_;
        geometry_msgs::msg::Point waypoint_1_;
        geometry_msgs::msg::Point waypoint_2_;
        geometry_msgs::msg::Twist cmd_vel_;
        double bot_yaw, bot_pitch, bot_roll ;
        bool yaw_fixed = false;
        double waypoint_angle;
        std::vector<double> waypoints ;
        double yaw_error, distance_error;
        double yaw_effort, distance_effort;
};

#endif
