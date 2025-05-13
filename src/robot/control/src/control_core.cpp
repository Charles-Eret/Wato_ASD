#include "control_core.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <algorithm>
#include <cmath>

ControlCore::ControlCore(const rclcpp::Logger& logger)
    : logger_(logger), lookahead_distance_(0.5), goal_tolerance_(0.1), linear_speed_(0.6) {}

void ControlCore::setPath(const nav_msgs::msg::Path::SharedPtr& path) {
    current_path_ = path;
}

void ControlCore::setOdom(const nav_msgs::msg::Odometry::SharedPtr& odom) {
    current_odom_ = odom;
}

std::optional<geometry_msgs::msg::Twist> ControlCore::computeControl() {
    if (!current_path_ || !current_odom_ || current_path_->poses.empty()) {
        RCLCPP_WARN(logger_, "Missing path or odometry data.");
        return std::nullopt;
    }

    geometry_msgs::msg::Pose current_pose = current_odom_->pose.pose;
    double robot_x = current_pose.position.x;
    double robot_y = current_pose.position.y;
    double robot_yaw = extractYaw(current_pose.orientation);

    for (const auto& pose : current_path_->poses) {
        double dx = pose.pose.position.x - robot_x;
        double dy = pose.pose.position.y - robot_y;
        double dist = std::sqrt(dx * dx + dy * dy);
        if (dist >= lookahead_distance_) {
            return computeVelocity(pose.pose, robot_x, robot_y, robot_yaw);
        }
    }

    // If we get here, we're near the goal.
    RCLCPP_INFO(logger_, "Goal reached or no valid lookahead point.");
    geometry_msgs::msg::Twist stop;
    stop.linear.x = 0.0;
    stop.angular.z = 0.0;
    return stop;
}

double ControlCore::extractYaw(const geometry_msgs::msg::Quaternion& q) const {
    tf2::Quaternion quat(q.x, q.y, q.z, q.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    return yaw;
}

std::optional<geometry_msgs::msg::Twist> ControlCore::computeVelocity(
    const geometry_msgs::msg::Pose& target,
    double robot_x, double robot_y, double robot_yaw) const {

    double dx = target.position.x - robot_x;
    double dy = target.position.y - robot_y;

    double target_angle = std::atan2(dy, dx);
    double angle_error = target_angle - robot_yaw;

    // Normalize angle to [-pi, pi]
    while (angle_error > M_PI) angle_error -= 2 * M_PI;
    while (angle_error < -M_PI) angle_error += 2 * M_PI;

    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = linear_speed_;
    cmd_vel.angular.z = 2.0 * angle_error;  // Gain can be tuned

    return cmd_vel;
}
