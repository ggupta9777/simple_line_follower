#ifndef LINE_FOLLOWER_HPP
#define LINE_FOLLOWER_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

class LineFollower : public rclcpp::Node {
    public :
        LineFollower();
        void setControlGain();
        void loadWaypoints();
        double calculateDistanceError();
        double calculateProportionalEffort(double error);
        double calculateDerivativeEffort(double error);
        void publishVelocity(double linear_x, double angular_z);
        void controlLoop();
        void odomCb(nav_msgs::msg::Odometry::SharedPtr msg);

        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::TimerBase::SharedPtr control_loop_timer_;
        geometry_msgs::msg::Point waypoint_1_;
        geometry_msgs::msg::Point waypoint_2_;
        double k_p, k_d, linear_vel;
        double bot_x, bot_y;
        std::vector<double> waypoints ;
        double prev_error = 0.0;
};

#endif
