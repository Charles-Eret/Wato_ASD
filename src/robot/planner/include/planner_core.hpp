#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/path.hpp>

namespace robot {

class PlannerCore {
public:
    explicit PlannerCore(const rclcpp::Logger& logger);

    nav_msgs::msg::Path getPath(
        const nav_msgs::msg::OccupancyGrid& map,
        const geometry_msgs::msg::PointStamped& goal,
        const geometry_msgs::msg::Pose& start_pose);

private:
    rclcpp::Logger logger_;

    // Map metadata
    unsigned int width_;
    unsigned int height_;
    float resolution_;
    float origin_x_;
    float origin_y_;
    const std::vector<int8_t>* map_data_;

    void setupMapMetadata(const nav_msgs::msg::OccupancyGrid& map);
    void toGrid(double wx, double wy, int& gx, int& gy) const;
    geometry_msgs::msg::PoseStamped toWorld(int gx, int gy) const;
    bool isValid(int x, int y) const;
    double heuristic(int x1, int y1, int x2, int y2) const;

    std::vector<std::pair<int, int>> runAStar(
        int sx, int sy,
        int gx, int gy,
        const nav_msgs::msg::OccupancyGrid& map);

    nav_msgs::msg::Path constructPath(
        const std::vector<std::pair<int, int>>& indices) const;
};

} // namespace robot
