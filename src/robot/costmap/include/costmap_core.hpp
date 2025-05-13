#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include "rclcpp/rclcpp.hpp"


namespace robot
{

class CostmapCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    CostmapCore(const rclcpp::Logger& logger);

    void updateCostmap(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    nav_msgs::msg::OccupancyGrid::SharedPtr getCostmap() const;
    
  private:
    double inflation_radius_;
    int inflation_cells_;

    std::shared_ptr<nav_msgs::msg::OccupancyGrid> costmap_data_;
    rclcpp::Logger logger_;

};

}  

#endif  