#ifndef NAIVE_LINE_FOLLOWER_HPP
#define NAIVE_LINE_FOLLOWER_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

class NaiveLineFollower : public rclcpp::Node {
    public :
        NaiveLineFollower();
        void setControlGain();
        void loadWaypoints();
        void goToGoal();
        void odomCb(nav_msgs::msg::Odometry::SharedPtr msg);

        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::TimerBase::SharedPtr control_loop_timer_;
        geometry_msgs::msg::Point waypoint_1_;
        geometry_msgs::msg::Point waypoint_2_;
        geometry_msgs::msg::Twist cmd_vel_;
        double k_p, k_d, linear_vel;
        double bot_x, bot_y;
        std::vector<double> waypoints ;
        double prev_error = 0.0;
};

#endif
