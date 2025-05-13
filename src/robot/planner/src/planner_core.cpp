#include "planner_core.hpp"
#include <queue>
#include <unordered_map>
#include <cmath>
#include <limits>

namespace robot {

struct Node {
    int x, y;
    double g, h;
    Node* parent;
    double f() const { return g + h; }
    bool operator>(const Node& other) const { return f() > other.f(); }
};

PlannerCore::PlannerCore(const rclcpp::Logger& logger)
: logger_(logger) {}

nav_msgs::msg::Path PlannerCore::getPath(
    const nav_msgs::msg::OccupancyGrid& map,
    const geometry_msgs::msg::PointStamped& goal,
    const geometry_msgs::msg::Pose& start_pose)
{
    setupMapMetadata(map);

    int sx, sy, gx, gy;
    toGrid(start_pose.position.x, start_pose.position.y, sx, sy);
    toGrid(goal.point.x, goal.point.y, gx, gy);

    auto path_indices = runAStar(sx, sy, gx, gy, map);
    return constructPath(path_indices);
}

void PlannerCore::setupMapMetadata(const nav_msgs::msg::OccupancyGrid& map) {
    width_ = map.info.width;
    height_ = map.info.height;
    resolution_ = map.info.resolution;
    origin_x_ = map.info.origin.position.x;
    origin_y_ = map.info.origin.position.y;
    map_data_ = &map.data;
}

void PlannerCore::toGrid(double wx, double wy, int& gx, int& gy) const {
    gx = static_cast<int>((wx - origin_x_) / resolution_);
    gy = static_cast<int>((wy - origin_y_) / resolution_);
}

geometry_msgs::msg::PoseStamped PlannerCore::toWorld(int gx, int gy) const {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.x = gx * resolution_ + origin_x_ + resolution_ / 2;
    pose.pose.position.y = gy * resolution_ + origin_y_ + resolution_ / 2;
    pose.header.stamp = rclcpp::Clock().now();
    return pose;
}

bool PlannerCore::isValid(int x, int y) const {
    int idx = y * width_ + x;
    return x >= 0 && x < static_cast<int>(width_) &&
           y >= 0 && y < static_cast<int>(height_) &&
           (*map_data_)[idx] >= 0 && (*map_data_)[idx] < 25;
}

double PlannerCore::heuristic(int x1, int y1, int x2, int y2) const {
    return std::hypot(x1 - x2, y1 - y2);
}

std::vector<std::pair<int, int>> PlannerCore::runAStar(int sx, int sy, int gx, int gy, const nav_msgs::msg::OccupancyGrid& map) {
    std::priority_queue<Node, std::vector<Node>, std::function<bool(Node, Node)>> open(
        [](const Node& a, const Node& b) { return a.f() > b.f(); });

    std::unordered_map<int, Node*> all_nodes;
    std::unordered_map<int, bool> closed;
    std::vector<std::pair<int, int>> directions = {{1,0},{-1,0},{0,1},{0,-1}};
    int start_idx = sy * width_ + sx;

    Node* start = new Node{sx, sy, 0.0, heuristic(sx, sy, gx, gy), nullptr};
    open.push(*start);
    all_nodes[start_idx] = start;

    Node* goal_node = nullptr;
    while (!open.empty()) {
        Node current = open.top(); open.pop();
        int cur_idx = current.y * width_ + current.x;
        if (closed[cur_idx]) continue;
        closed[cur_idx] = true;

        if (current.x == gx && current.y == gy) {
            goal_node = new Node(current);
            break;
        }

        for (auto [dx, dy] : directions) {
            int nx = current.x + dx;
            int ny = current.y + dy;
            if (!isValid(nx, ny)) continue;

            int nid = ny * width_ + nx;
            double ng = current.g + resolution_;
            if (!all_nodes.count(nid) || ng < all_nodes[nid]->g) {
                Node* neighbor = new Node{nx, ny, ng, heuristic(nx, ny, gx, gy), all_nodes[cur_idx]};
                all_nodes[nid] = neighbor;
                open.push(*neighbor);
            }
        }
    }

    std::vector<std::pair<int, int>> path;
    if (goal_node) {
        for (Node* n = goal_node; n != nullptr; n = n->parent) {
            path.emplace_back(n->x, n->y);
        }
        std::reverse(path.begin(), path.end());
    } else {
        RCLCPP_WARN(logger_, "A* failed to find a path.");
    }

    for (auto& [_, node] : all_nodes) delete node;
    delete goal_node;
    return path;
}

nav_msgs::msg::Path PlannerCore::constructPath(const std::vector<std::pair<int, int>>& indices) const {
    nav_msgs::msg::Path path;
    path.header.stamp = rclcpp::Clock().now();
    path.header.frame_id = "map";
    for (auto [x, y] : indices) {
        path.poses.push_back(toWorld(x, y));
    }
    return path;
}

} // namespace robot
