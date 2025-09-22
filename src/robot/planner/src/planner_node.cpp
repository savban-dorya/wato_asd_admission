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
        "/map_memory", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
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
    state_ = State::WAITING_FOR_GOAL;  // Initialize state
} 

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    current_map_ = *msg;
    RCLCPP_INFO(this->get_logger(), "Map received: %dx%d, resolution: %.3f", 
                current_map_.info.width, current_map_.info.height, current_map_.info.resolution);
    
    if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
        planPath();
    }
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
    goal_ = *msg;
    goal_received_ = true;
    state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
    
    RCLCPP_INFO(this->get_logger(), "Goal received: (%.2f, %.2f)", 
                goal_.point.x, goal_.point.y);
    
    planPath();
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    robot_pose_ = msg->pose.pose;
    
    // Debug: Print robot position occasionally
    static int counter = 0;
    if (counter++ % 20 == 0) {  // Every 20 messages
        //RCLCPP_INFO(this->get_logger(), "Robot position: (%.2f, %.2f)", 
        //            robot_pose_.position.x, robot_pose_.position.y);
    }
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
    double distance = std::sqrt(dx * dx + dy * dy);
    
    // RCLCPP_DEBUG(this->get_logger(), "Distance to goal: %.3f", distance);
    return distance < 0.5; // Threshold for reaching the goal
}
 
bool PlannerNode::isTraversable(const CellIndex& cell) {
    if (cell.x < 0 || cell.x >= static_cast<int>(current_map_.info.width) ||
        cell.y < 0 || cell.y >= static_cast<int>(current_map_.info.height)) {
        return false;
    }
    
    int index = cell.y * current_map_.info.width + cell.x;
    if (index >= static_cast<int>(current_map_.data.size())) {
        return false;
    }
    
    // Check for unknown (-1), occupied (100), or high probability cells
    int cell_value = current_map_.data[index];
    return cell_value == 0;
}

double PlannerNode::calculateHeuristic(const CellIndex& start, const CellIndex& goal) {
    // Use Euclidean distance for better heuristic
    double dx = goal.x - start.x;
    double dy = goal.y - start.y;
    return std::sqrt(dx * dx + dy * dy);
}

double PlannerNode::getMovementCost(const CellIndex& from, const CellIndex& to) {
    int dx = std::abs(to.x - from.x);
    int dy = std::abs(to.y - from.y);
    return (dx + dy > 1) ? 1.414 : 1.0; // Diagonal movement costs sqrt(2)
}

PlannerNode::CellIndex PlannerNode::worldToGrid(const geometry_msgs::msg::Point& point) {
    int x = static_cast<int>((point.x - current_map_.info.origin.position.x) / 
                            current_map_.info.resolution);
    int y = static_cast<int>((point.y - current_map_.info.origin.position.y) / 
                            current_map_.info.resolution);
    
    //RCLCPP_DEBUG(this->get_logger(), "World (%.2f, %.2f) -> Grid (%d, %d)", 
    //            point.x, point.y, x, y);
    
    return CellIndex(x, y);
}

geometry_msgs::msg::PoseStamped PlannerNode::gridToWorld(const CellIndex& cell) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "sim_world";
    pose.header.stamp = this->get_clock()->now();
    
    pose.pose.position.x = cell.x * current_map_.info.resolution + 
                            current_map_.info.origin.position.x;
    pose.pose.position.y = cell.y * current_map_.info.resolution + 
                            current_map_.info.origin.position.y;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.w = 1.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    
    return pose;
}

void PlannerNode::planPath() {
    if (!goal_received_) {
        RCLCPP_WARN(this->get_logger(), "Cannot plan path: No goal received!");
        return;
    }
    
    if (current_map_.data.empty()) {
        RCLCPP_WARN(this->get_logger(), "Cannot plan path: Map data is empty!");
        return;
    }
    
    RCLCPP_INFO(this->get_logger(), "Planning path...");

    // Convert start and goal positions to grid coordinates
    CellIndex start_cell = worldToGrid(robot_pose_.position);
    CellIndex goal_cell = worldToGrid(goal_.point);

    //RCLCPP_INFO(this->get_logger(), "Start cell: (%d, %d), Goal cell: (%d, %d)", 
    //            start_cell.x, start_cell.y, goal_cell.x, goal_cell.y);

    // Check if start and goal positions are valid
    if (!isTraversable(start_cell)) {
        RCLCPP_ERROR(this->get_logger(), "Start position is not traversable!");
        return;
    }
    
    if (!isTraversable(goal_cell)) {
        RCLCPP_ERROR(this->get_logger(), "Goal position is not traversable!");
        return;
    }

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
    int iterations = 0;
    const int max_iterations = 10000;  // Prevent infinite loops

    // Main A* loop
    while (!openSet.empty() && iterations < max_iterations) {
        iterations++;
        current = openSet.top();
        openSet.pop();

        // Check if we've reached the goal
        if (current->index == goal_cell) {
            pathFound = true;
            // RCLCPP_INFO(this->get_logger(), "Path found after %d iterations", iterations);
            break;
        }

        closedSet.insert(current->index);

        // Check all 8 neighbors
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

    if (iterations >= max_iterations) {
        RCLCPP_WARN(this->get_logger(), "A* algorithm reached maximum iterations (%d)", max_iterations);
    }

    // Create path message
    nav_msgs::msg::Path path;
    path.header.stamp = this->get_clock()->now();
    path.header.frame_id = "sim_world";

    // Reconstruct and publish path if one was found
    if (pathFound && current != nullptr) {
        std::vector<geometry_msgs::msg::PoseStamped> poses;
        AStarNode* path_node = current;
        
        while (path_node != nullptr) {
            poses.push_back(gridToWorld(path_node->index));
            path_node = path_node->parent;
        }

        // Reverse the path to go from start to goal
        std::reverse(poses.begin(), poses.end());
        path.poses = poses;

        // RCLCPP_INFO(this->get_logger(), "Path found with %zu waypoints", poses.size());
    } else {
        RCLCPP_WARN(this->get_logger(), "No path found to goal");
        // Still publish empty path to indicate no path found
    }

    // Cleanup allocated nodes
    for (auto& pair : allNodes) {
        delete pair.second;
    }

    // Always publish the path (even if empty)
    path_pub_->publish(path);
    RCLCPP_INFO(this->get_logger(), "Path published with %zu poses", path.poses.size());
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PlannerNode>());
    rclcpp::shutdown();
    return 0;
}