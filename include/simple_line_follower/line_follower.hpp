#ifndef LINE_FOLLOWER_HPP
#define LINE_FOLLOWER_HPP

#include <algorithm> 
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "nav_msgs/msg/odometry.hpp"

class LineFollower : public rclcpp::Node {
    public :
        LineFollower();
        void setControlGain();
        void loadWaypoints();
        double calculateDistanceError(const geometry_msgs::msg::TransformStamped& transform);
        double calculateDistanceEffort(double error);
        void publishVelocity(double linear_x, double angular_z);
        void controlLoop();

        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
        rclcpp::TimerBase::SharedPtr control_loop_timer_;
        geometry_msgs::msg::Point waypoint_1_;
        geometry_msgs::msg::Point waypoint_2_;
        double k_p, linear_vel;
        double bot_yaw, bot_pitch, bot_roll ;
        double waypoint_angle;
        std::vector<double> waypoints ;
        double distance_error, distance_effort;
};

#endif
