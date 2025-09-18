#include "planner_node.hpp"

#include <queue>
#include <unordered_map>
#include <vector>
#include <cmath>

PlannerNode::PlannerNode() : Node("planner"), planner_(robot::PlannerCore(this->get_logger())) {

  // TODO: Check if I copy pasted this in the correct area
  // yeah probably

class PlannerNode : public rclcpp::Node {
public:
    PlannerNode() : Node("planner_node"), state_(State::WAITING_FOR_GOAL) {

        // Remember to boot up docker container (with ./watod --setup-dev-env robot) to load all dependencies

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
    }
 
private:
    enum class State { WAITING_FOR_GOAL, WAITING_FOR_ROBOT_TO_REACH_GOAL };
    State state_;
 
    // Subscribers and Publisher
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
 
    // Data Storage
    nav_msgs::msg::OccupancyGrid current_map_;
    geometry_msgs::msg::PointStamped goal_;
    geometry_msgs::msg::Pose robot_pose_;
 
    bool goal_received_ = false;
 
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        current_map_ = *msg;
        if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
            planPath();
        }
    }
 
    void goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        goal_ = *msg;
        goal_received_ = true;
        state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
        planPath();
    }
 
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        robot_pose_ = msg->pose.pose;
    }
 
    void timerCallback() {
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
 
    bool goalReached() {
        double dx = goal_.point.x - robot_pose_.position.x;
        double dy = goal_.point.y - robot_pose_.position.y;
        return std::sqrt(dx * dx + dy * dy) < 0.5; // Threshold for reaching the goal
    }
 

    // Helper function to get adjacent nodes 
    std::vector<CellIndex> getAdjacentCells(const CellIndex& current) {
        std::vector<CellIndex> neighbors;
        // Define the 8 possible movements (8-connected grid)
        const int dx[] = {-1, -1, -1,  0,  0,  1, 1, 1};
        const int dy[] = {-1,  0,  1, -1,  1, -1, 0, 1};
        
        for (int i = 0; i < 8; ++i) {
            CellIndex neighbor(current.x + dx[i], current.y + dy[i]);
            
            // Check if the neighbor is within map bounds
            if (neighbor.x >= 0 && neighbor.x < current_map_.info.width &&
                neighbor.y >= 0 && neighbor.y < current_map_.info.height) {
                // Check if the cell is not occupied (assuming occupancy > 50 means occupied)
                int index = neighbor.y * current_map_.info.width + neighbor.x;
                if (current_map_.data[index] < 50) {
                    neighbors.push_back(neighbor);
                }
            }
        }
        return neighbors;
    }

    // Helper function to calculate movement cost
    double getMovementCost(const CellIndex& from, const CellIndex& to) {
        int dx = std::abs(to.x - from.x);
        int dy = std::abs(to.y - from.y);
        
        // If diagonal movement (both dx and dy are 1)
        if (dx == 1 && dy == 1) {
            return 1.414; // √2
        }  
        return 1.0; // Straight movement
    }

    //IDK HOW TF THIS WORKS
    // Helper function to calculate heuristic cost (Manhattan distance)
    double calculateHeuristic(const CellIndex& current, const CellIndex& goal) {
        return std::abs(goal.x - current.x) + std::abs(goal.y - current.y);
    }

    // Helper function to convert world coordinates to grid coordinates
    CellIndex worldToGrid(const geometry_msgs::msg::Point& point) {
        int x = static_cast<int>((point.x - current_map_.info.origin.position.x) / 
                                current_map_.info.resolution);
        int y = static_cast<int>((point.y - current_map_.info.origin.position.y) / 
                                current_map_.info.resolution);
        return CellIndex(x, y);
    }

    // Helper function to convert grid coordinates to world coordinates
    geometry_msgs::msg::PoseStamped gridToWorld(const CellIndex& cell) {
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



    void planPath() {
        if (!goal_received_ || current_map_.data.empty()) {
            RCLCPP_WARN(this->get_logger(), "Cannot plan path: Missing map or goal!");
            return;
        }

        // Convert start and goal positions to grid coordinates
        CellIndex start = worldToGrid(robot_pose_.position);
        CellIndex goal = worldToGrid(goal_.point);

        // Initialize the path message
        nav_msgs::msg::Path path;
        path.header.stamp = this->get_clock()->now();
        path.header.frame_id = "map";

        // A* algorithm data structures
        std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> openSet;
        std::unordered_map<CellIndex, CellIndex, CellIndexHash> cameFrom;
        std::unordered_map<CellIndex, double, CellIndexHash> gScore;
        
        // Initialize start node
        openSet.push(AStarNode(start, calculateHeuristic(start, goal)));
        gScore[start] = 0;

        while (!openSet.empty()) {
            CellIndex current = openSet.top().index;
            openSet.pop();

            // Check if we reached the goal
            if (current == goal) {
                // Reconstruct path
                std::vector<CellIndex> pathIndices;
                while (current != start) {
                    pathIndices.push_back(current);
                    current = cameFrom[current];
                }
                pathIndices.push_back(start);
                
                // Convert path to world coordinates
                for (auto it = pathIndices.rbegin(); it != pathIndices.rend(); ++it) {
                    path.poses.push_back(gridToWorld(*it));
                }
                
                path_pub_->publish(path);
                return;
            }

            // Check all neighbors
            for (const auto& neighbor : getAdjacentCells(current)) {
                double tentative_gScore = gScore[current] + getMovementCost(current, neighbor);
                
                if (!gScore.count(neighbor) || tentative_gScore < gScore[neighbor]) {
                    // This path is better than any previous one
                    cameFrom[neighbor] = current;
                    gScore[neighbor] = tentative_gScore;
                    double fScore = tentative_gScore + calculateHeuristic(neighbor, goal);
                    openSet.push(AStarNode(neighbor, fScore));
                }
            }
        }

        RCLCPP_WARN(this->get_logger(), "No path found to goal!");
    }



        // A* Implementation (pseudo-code)
        nav_msgs::msg::Path path;
        path.header.stamp = this->get_clock()->now();
        path.header.frame_id = "map";
 
        // Compute path using A* on current_map_
        
        // create a "start Node" and an "end node" for current map
        // pseudo-code
        AStarNode end_node = goal_.convertToNode();
        AStarNode start_node = current_locationIThink.convertToNode();


        // How to access all the nodes adjacent to your start node?
        
        // what is the standard distance of a "node" in this case?
            // current res is 0.1m, so diagonal nodes are 0.1414
            //F-score is * 10, so 


        // How to know if a node is next to current or diagonal to current?
            // hardcode the array to scan left to right, top to down, and 0,2,5,7 is 1.4, rest are 1
        


        // while you didn't reach the end goal, look thru each node, find lowest F score, explore there
        // recalculate all the scores with the new nodes close to it
        // 

        // Fill path.poses with the resulting waypoints.
 
        path_pub_->publish(path);
    }
};

}



int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
