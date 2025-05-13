#include "control_node.hpp"

ControlNode::ControlNode() : Node("control_node"), controller_(this->get_logger()) {
    path_subscriber_ = this->create_subscription<nav_msgs::msg::Path>(
        "/path", 10, std::bind(&ControlNode::pathCallback, this, std::placeholders::_1));

    odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom/filtered", 10, std::bind(&ControlNode::odomCallback, this, std::placeholders::_1));

    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    control_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), std::bind(&ControlNode::controlLoop, this));
}

void ControlNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
    controller_.setPath(msg);
}

void ControlNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    controller_.setOdom(msg);
}

void ControlNode::controlLoop() {
    auto cmd_vel = controller_.computeControl();
    if (cmd_vel.has_value()) {
        cmd_vel_publisher_->publish(cmd_vel.value());
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControlNode>());
    rclcpp::shutdown();
    return 0;
}