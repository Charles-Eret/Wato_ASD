#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include "nav_msgs/msg/occupancy_grid.hpp"

#include "rclcpp/rclcpp.hpp"

namespace robot
{

class MapMemoryCore {
  public:
    explicit MapMemoryCore(const rclcpp::Logger& logger);
    void updateCurrentCostmap(const std::shared_ptr<nav_msgs::msg::OccupancyGrid>& currentCostmap);
    void updateGlobalCostmap(double robot_x, double robot_y, double robot_theta);
    nav_msgs::msg::OccupancyGrid::SharedPtr getGlobalCostmap() const;

  private:
    rclcpp::Logger logger_;
    std::shared_ptr<nav_msgs::msg::OccupancyGrid> current_costmap_;
    std::shared_ptr<nav_msgs::msg::OccupancyGrid> global_costmap_;
};

}  

#endif  
