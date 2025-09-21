#include "planner_node.hpp"
#include <vector>
#include <limits>
#include <algorithm>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <chrono>
#include <cmath> 

PlannerNode::PlannerNode() : Node("planner"), planner_(robot::PlannerCore(this->get_logger())) {
    // Subscribers
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));

    // Publisher
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

    // Timer
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500), std::bind(&PlannerNode::timerCallback, this));

    goal_received_ = false;
} 

// Map Call back function stores newest map and if pursueing goal triggers the call
void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    current_map_ = *msg;
    if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
        planPath();
    }
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
    goal_ = *msg;
    goal_received_ = true;
    state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
    planPath();
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    robot_pose_ = msg->pose.pose;
}
 
void PlannerNode::timerCallback() {
    if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
        if (goalReached()) {
            RCLCPP_INFO(this->get_logger(), "Goal reached!");
            state_ = State::WAITING_FOR_GOAL;
        } else {
            RCLCPP_INFO(this->get_logger(), "Replanning due to timeout or progress...");
            planPath();
        }
    }
}
 
bool PlannerNode::goalReached() {
    double dx = goal_.point.x - robot_pose_.position.x;
    double dy = goal_.point.y - robot_pose_.position.y;
    return std::sqrt(dx * dx + dy * dy) < 0.5; // Threshold for reaching the goal
}
 
// Helper function to check if a cell is traversable
bool PlannerNode::isTraversable(const CellIndex& cell) {
    if (cell.x < 0 || cell.x >= current_map_.info.width ||
        cell.y < 0 || cell.y >= current_map_.info.height) {
        return false;
    }
    int index = cell.y * current_map_.info.width + cell.x;
    return current_map_.data[index] < 50; // Assuming < 50 means free space
}

// Calculate Manhattan distance heuristic
double PlannerNode::calculateHeuristic(const CellIndex& start, const CellIndex& goal) {
    return std::abs(goal.x - start.x) + std::abs(goal.y - start.y);
}

// Get movement cost between adjacent cells
double PlannerNode::getMovementCost(const CellIndex& from, const CellIndex& to) {
    int dx = std::abs(to.x - from.x);
    int dy = std::abs(to.y - from.y);
    return (dx + dy > 1) ? 1.414 : 1.0; // Diagonal movement costs sqrt(2)
}

// Convert world coordinates to grid coordinates
PlannerNode::CellIndex PlannerNode::worldToGrid(const geometry_msgs::msg::Point& point) {
    int x = static_cast<int>((point.x - current_map_.info.origin.position.x) / 
                            current_map_.info.resolution);
    int y = static_cast<int>((point.y - current_map_.info.origin.position.y) / 
                            current_map_.info.resolution);
    return CellIndex(x, y);
}

// Convert grid coordinates to world coordinates
geometry_msgs::msg::PoseStamped PlannerNode::gridToWorld(const CellIndex& cell) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = this->get_clock()->now();
    
    pose.pose.position.x = cell.x * current_map_.info.resolution + 
                            current_map_.info.origin.position.x;
    pose.pose.position.y = cell.y * current_map_.info.resolution + 
                            current_map_.info.origin.position.y;
    pose.pose.orientation.w = 1.0;
    return pose;
}

void PlannerNode::planPath() {
    if (!goal_received_ || current_map_.data.empty()) {
        RCLCPP_WARN(this->get_logger(), "Cannot plan path: Missing map or goal!");
        return;
    }
        
        
    nav_msgs::msg::Path path;
    path.header.stamp = this->get_clock()->now();
    path.header.frame_id = "map";
    RCLCPP_INFO(this->get_logger(), "map data found and goal received");

    // Convert start and goal positions to grid coordinates
    CellIndex start_cell = worldToGrid(robot_pose_.position);
    CellIndex goal_cell = worldToGrid(goal_.point);

    // Initialize data structures for A*
    std::priority_queue<AStarNode*, std::vector<AStarNode*>, CompareF> openSet;
    std::unordered_map<CellIndex, AStarNode*, CellIndexHash> allNodes;
    std::unordered_set<CellIndex, CellIndexHash> closedSet;

    // Create and add start node
    AStarNode* start_node = new AStarNode(start_cell, 0, 
        calculateHeuristic(start_cell, goal_cell));
    openSet.push(start_node);
    allNodes[start_cell] = start_node;

    bool pathFound = false;
    AStarNode* current = nullptr;

    // Main A* loop
    while (!openSet.empty()) {
        current = openSet.top();
        openSet.pop();

        // Check if we've reached the goal
        if (current->index == goal_cell) {
            pathFound = true;
            break;
        }

        closedSet.insert(current->index);

        // Check all neighbors
        for (int dx = -1; dx <= 1; dx++) {
            for (int dy = -1; dy <= 1; dy++) {
                if (dx == 0 && dy == 0) continue;

                CellIndex neighbor_idx(current->index.x + dx, current->index.y + dy);

                // Skip if not traversable or already in closed set
                if (!isTraversable(neighbor_idx) || 
                    closedSet.find(neighbor_idx) != closedSet.end()) {
                    continue;
                }

                // Calculate tentative g score
                double tentative_g = current->g_score + 
                    getMovementCost(current->index, neighbor_idx);

                AStarNode* neighbor;
                bool is_new = false;

                // Create new neighbor node if it doesn't exist
                if (allNodes.find(neighbor_idx) == allNodes.end()) {
                    neighbor = new AStarNode(neighbor_idx, tentative_g,
                        calculateHeuristic(neighbor_idx, goal_cell));
                    allNodes[neighbor_idx] = neighbor;
                    is_new = true;
                } else {
                    neighbor = allNodes[neighbor_idx];
                }

                // Update neighbor if we found a better path
                if (is_new || tentative_g < neighbor->g_score) {
                    neighbor->parent = current;
                    neighbor->g_score = tentative_g;
                    neighbor->f_score = tentative_g + neighbor->h_score;

                    if (is_new) {
                        openSet.push(neighbor);
                    }
                }
            }
        }
    }

    // Reconstruct and publish path if one was found
    if (pathFound) {
        std::vector<geometry_msgs::msg::PoseStamped> poses;
        while (current != nullptr) {
            poses.push_back(gridToWorld(current->index));
            current = current->parent;
        }

        // Reverse the path to go from start to goal
        std::reverse(poses.begin(), poses.end());
        path.poses = poses;

        RCLCPP_INFO(this->get_logger(), "Path found with %zu waypoints", poses.size());
    } else {
        RCLCPP_WARN(this->get_logger(), "No path found to goal");
    }

    // Cleanup allocated nodes
    for (auto& pair : allNodes) {
        delete pair.second;
    }  
    // Compute path using A* on current_map_
    // Fill path.poses with the resulting waypoints.
    //pseudocode
    // Show start node and end node on foxglove
    // these are all 30x30 cells, so each cell is 5m (im assuming)

    // int[] map_data = map_sub_.data;

    
    // getting covariance matrix
    // odom_sub_ is a subscription handle, not the data
    // std::array<double, 36> robot_pose = robot_pose_.covariance;
    // int robot_x = int(robot_pose.x);
    // int robot_y = int(robot_pose.y);
    // // round position to nearest int and assign it a cell
    // //cells are 0,299 x and y, and coordinates range too
    // // to find correct cell, convert robot pose coords to a specific cell in map memory
    // // using map from callback to access info
    // double resolution = current_map_->info.resolution;
    // int width = current_map_->info.width;
    // int height = current_map_->info.height;


    // Cleanup allocated nodes
    for (auto& pair : allNodes) {
        delete pair.second;
    }

    // publish the path
    path_pub_->publish(path);
}







int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}