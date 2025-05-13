#include "map_memory_core.hpp"

namespace robot
{

MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger) 
  : logger_(logger),
  current_costmap_(std::make_shared<nav_msgs::msg::OccupancyGrid>()),
  global_costmap_(std::make_shared<nav_msgs::msg::OccupancyGrid>()) {
    global_costmap_->info.resolution = 0.1;
    global_costmap_->info.width = 300;
    global_costmap_->info.height = 300;
    global_costmap_->info.origin.position.x = -15.0;
    global_costmap_->info.origin.position.y = -15.0;
    global_costmap_->info.origin.orientation.w = 1.0;
    global_costmap_->data.assign(300 * 300, 0); 
  }

void MapMemoryCore::updateCurrentCostmap(const std::shared_ptr<nav_msgs::msg::OccupancyGrid>& currentCostmap) {
  //update the current costmap with the new data
  if (currentCostmap) {
    current_costmap_ = currentCostmap;
    //RCLCPP_INFO(logger_, "Updated current costmap");
  }
}

void MapMemoryCore::updateGlobalCostmap(double robot_x, double robot_y, double robot_theta) {
  // Merge the current costmap into the global costmap
  if (!current_costmap_) return;

  const auto& local = current_costmap_->info;
  const auto& local_data = current_costmap_->data;

  for (unsigned int j = 0; j < local.height; ++j) {
    for (unsigned int i = 0; i < local.width; ++i) {
      int8_t occ_val = local_data[j * local.width + i];
      if (occ_val < 0) continue;

      // local coordinates (relative to local costmap origin)
      double lx = local.origin.position.x + (i + 0.5) * local.resolution;
      double ly = local.origin.position.y + (j + 0.5) * local.resolution;

      // transform into global frame using robot pose
      double cos_t = std::cos(robot_theta);
      double sin_t = std::sin(robot_theta);
      double wx = robot_x + (lx * cos_t - ly * sin_t);
      double wy = robot_y + (lx * sin_t + ly * cos_t);

      // convert to global grid index
      double gx_origin = global_costmap_->info.origin.position.x;
      double gy_origin = global_costmap_->info.origin.position.y;
      double res = global_costmap_->info.resolution;

      int gx = static_cast<int>((wx - gx_origin) / res);
      int gy = static_cast<int>((wy - gy_origin) / res);

      if (gx < 0 || gx >= static_cast<int>(global_costmap_->info.width) ||
          gy < 0 || gy >= static_cast<int>(global_costmap_->info.height))
        continue;

      int idx = gy * global_costmap_->info.width + gx;
      int8_t& global_val = global_costmap_->data[idx];

      int merged = std::max(static_cast<int>(global_val < 0 ? 0 : global_val), static_cast<int>(occ_val));

      global_val = static_cast<int8_t>(merged);
    }
  }

  //RCLCPP_INFO(logger_, "Transformed and merged local costmap into global map.");
}


nav_msgs::msg::OccupancyGrid::SharedPtr robot::MapMemoryCore::getGlobalCostmap() const {
    return global_costmap_;
}

} 
