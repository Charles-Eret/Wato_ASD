#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())), 
current_robot_x_(0.0), current_robot_y_(0.0), current_robot_theta_(0.0),
last_robot_x_(0.0), last_robot_y_(0.0)
{
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/costmap", 
        10, 
        std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 
      10, 
      std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));
  timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500), std::bind(&MapMemoryNode::distCheck, this));
  global_costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map",10);
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  map_memory_.updateCurrentCostmap(msg);
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  current_robot_x_ = msg->pose.pose.position.x;
  current_robot_y_ = msg->pose.pose.position.y;

  // Convert quaternion to yaw
  double qx = msg->pose.pose.orientation.x;
  double qy = msg->pose.pose.orientation.y;
  double qz = msg->pose.pose.orientation.z;
  double qw = msg->pose.pose.orientation.w;
  tf2::Quaternion q(qx, qy, qz, qw);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  current_robot_theta_ = yaw;
  //RCLCPP_INFO(this->get_logger(), "Updated odom position.");
}

void MapMemoryNode::distCheck() {
  double dx = current_robot_x_ - last_robot_x_;
  double dy = current_robot_y_ - last_robot_y_;
  double distance = std::sqrt(dx * dx + dy * dy);
  //if the robot has moved more than 1.5 meters, update the global costmap
  if (distance > 1.5) {
    last_robot_x_ = current_robot_x_;
    last_robot_y_ = current_robot_y_;
    map_memory_.updateGlobalCostmap(current_robot_x_, current_robot_y_, current_robot_theta_);
    nav_msgs::msg::OccupancyGrid map_msg = *map_memory_.getGlobalCostmap();
    map_msg.header.stamp = this->now();
    map_msg.header.frame_id = "sim_world";
    global_costmap_pub_->publish(map_msg);
    RCLCPP_INFO(this->get_logger(), "Distance check passed, updated last position.");
  } 
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
