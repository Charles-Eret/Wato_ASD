#include <chrono>
#include <memory>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include "costmap_node.hpp"
 
CostmapNode::CostmapNode() : Node("costmap"),
  costmap_(robot::CostmapCore(this->get_logger())) {
    laserScan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar", 10, std::bind(&CostmapNode::laserScanCallback, this, std::placeholders::_1));
    costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
}
 
void CostmapNode::laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  //update the costmap with the laser scan data
  costmap_.updateCostmap(msg);
  std::shared_ptr<nav_msgs::msg::OccupancyGrid> costmap_data = costmap_.getCostmap();
  costmap_data->header.stamp = msg->header.stamp;      
  costmap_data->header.frame_id = msg->header.frame_id;
  costmap_pub_->publish(*costmap_data);
  //RCLCPP_INFO(this->get_logger(), "Costmap published");
}
 
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}