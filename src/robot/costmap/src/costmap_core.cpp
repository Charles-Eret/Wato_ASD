#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include "costmap_core.hpp"

namespace robot
{

CostmapCore::CostmapCore(const rclcpp::Logger& logger)
    : logger_(logger),
      costmap_data_(std::make_shared<nav_msgs::msg::OccupancyGrid>()) {
    costmap_data_->info.resolution = 0.1;
    costmap_data_->info.width = 100;
    costmap_data_->info.height = 100;
    costmap_data_->info.origin.position.x = -5.0;
    costmap_data_->info.origin.position.y = -5.0;
    costmap_data_->info.origin.orientation.w = 1.0;
    costmap_data_->data.assign(costmap_data_->info.width * costmap_data_->info.height, -1);

    inflation_radius_ = 1.0;
    inflation_cells_ = static_cast<int>(inflation_radius_ / costmap_data_->info.resolution);

    RCLCPP_INFO(logger_, "2D costmap grid initialized: resolution=%.2f, size=%dx%d, inflation=%d cells",
            costmap_data_->info.resolution,
            costmap_data_->info.width,
            costmap_data_->info.height,
            inflation_cells_);
}

void CostmapCore::updateCostmap(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    //update the costmap with the laser scan data
    int width = costmap_data_->info.width;
    int height = costmap_data_->info.height;
    double resolution = costmap_data_->info.resolution;
    double origin_x = costmap_data_->info.origin.position.x;
    double origin_y = costmap_data_->info.origin.position.y;

    // Reset grid
    std::vector<std::vector<int8_t>> grid(height, std::vector<int8_t>(width, 0));

    double angle = msg->angle_min;
    for (size_t i = 0; i < msg->ranges.size(); ++i, angle += msg->angle_increment) {
        double range = msg->ranges[i];
        if (range >= msg->range_min && range <= msg->range_max) {
            // Scan points are already in the fixed global frame
            double x = range * std::cos(angle);
            double y = range * std::sin(angle);

            // Convert to grid coordinates
            int grid_x = static_cast<int>((x - origin_x) / resolution);
            int grid_y = static_cast<int>((y - origin_y) / resolution);

            if (grid_x >= 0 && grid_x < width && grid_y >= 0 && grid_y < height) {
                grid[grid_y][grid_x] = 100;

                // Inflate
                for (int dy = -inflation_cells_; dy <= inflation_cells_; ++dy) {
                    for (int dx = -inflation_cells_; dx <= inflation_cells_; ++dx) {
                        int nx = grid_x + dx;
                        int ny = grid_y + dy;
                        if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                            double distance = std::hypot(dx, dy) * resolution;
                            if (distance <= inflation_radius_) {
                                int inflated_cost = static_cast<int>(100 * (1.0 - distance / inflation_radius_));
                                if (inflated_cost > grid[ny][nx]) {
                                    grid[ny][nx] = inflated_cost;
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    // Flatten to 1D costmap
    costmap_data_->data.clear();
    costmap_data_->data.reserve(width * height);
    for (const auto& row : grid) {
        costmap_data_->data.insert(costmap_data_->data.end(), row.begin(), row.end());
    }
}


nav_msgs::msg::OccupancyGrid::SharedPtr robot::CostmapCore::getCostmap() const {
    return costmap_data_;
}

}

