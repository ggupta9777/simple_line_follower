#ifndef LINE_FOLLOWER_HPP
#define LINE_FOLLOWER_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "yaml-cpp/yaml.h"
#include <cmath>

class LineFollower : public rclcpp::Node {
    public :
        LineFollower();
        void loadWaypoints();
        double calculateDistanceError(const geometry_msgs::msg::TransformStamped& transform);
        double calculateDistanceEffort(double error);
        void quaternionToEuler(const tf2::Quaternion &q, double &roll, double &pitch, double &yaw);
        double calculateYawError(const geometry_msgs::msg::TransformStamped& transform);
        double calculateYawEffort(double error);
        void publishVelocity(double linear_x, double angular_z);
        void controlLoop();

        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
        rclcpp::TimerBase::SharedPtr control_loop_timer_;
        geometry_msgs::msg::Point waypoint_1_;
        geometry_msgs::msg::Point waypoint_2_;
        double bot_yaw, bot_pitch, bot_roll ;
        bool yaw_fixed = false;
        double waypoint_angle;
        std::vector<double> waypoints ;
        double yaw_error, distance_error;
        double yaw_effort, distance_effort;
};

#endif
