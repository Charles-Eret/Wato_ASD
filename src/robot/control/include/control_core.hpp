#pragma once

#include <rclcpp/logger.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <optional>

class ControlCore {
public:
    explicit ControlCore(const rclcpp::Logger& logger);

    void setPath(const nav_msgs::msg::Path::SharedPtr& path);
    void setOdom(const nav_msgs::msg::Odometry::SharedPtr& odom);

    std::optional<geometry_msgs::msg::Twist> computeControl();

private:
    double extractYaw(const geometry_msgs::msg::Quaternion& q) const;
    std::optional<geometry_msgs::msg::Twist> computeVelocity(
        const geometry_msgs::msg::Pose& target,
        double robot_x,
        double robot_y,
        double robot_yaw) const;

    nav_msgs::msg::Path::SharedPtr current_path_;
    nav_msgs::msg::Odometry::SharedPtr current_odom_;

    rclcpp::Logger logger_;
    double lookahead_distance_;
    double goal_tolerance_;
    double linear_speed_;
};
