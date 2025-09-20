#include "planner_node.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include <vector>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <cmath>

 // ------------------- Supporting Structures -------------------
  
  // 2D grid index
  struct CellIndex
  {
    int x;
    int y;
  
    CellIndex(int xx, int yy) : x(xx), y(yy) {}
    CellIndex() : x(0), y(0) {}
  
    bool operator==(const CellIndex &other) const
    {
      return (x == other.x && y == other.y);
    }
  
    bool operator!=(const CellIndex &other) const
    {
      return (x != other.x || y != other.y);
    }
  };
  
  // Hash function for CellIndex so it can be used in std::unordered_map
  struct CellIndexHash
  {
    std::size_t operator()(const CellIndex &idx) const
    {
      // A simple hash combining x and y
      return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
    }
  };
  
  // Structure representing a node in the A* open set
  struct AStarNode
  {
    CellIndex index;
    double f_score;  // f = g + h
    double g_score;  // cost from start
    double h_score;  // heuristic to goal
    AStarNode* parent;
  
    AStarNode(CellIndex idx, double g, double h) 
      : index(idx), g_score(g), h_score(h), f_score(g + h), parent(nullptr) {}
  };
  
  // Comparator for the priority queue (min-heap by f_score)
  struct CompareF
  {
    bool operator()(const AStarNode &a, const AStarNode &b)
    {
      // We want the node with the smallest f_score on top
      return a.f_score > b.f_score;
    }
  };



PlannerNode::PlannerNode() : Node("planner"), planner_(robot::PlannerCore(this->get_logger())) {

  class PlannerNode : public rclcpp::Node {
  public:
      PlannerNode() : Node("planner_node"), state_(State::WAITING_FOR_GOAL) {
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
 
    // Helper function to check if a cell is traversable
    bool isTraversable(const CellIndex& cell) {
        if (cell.x < 0 || cell.x >= current_map_.info.width ||
            cell.y < 0 || cell.y >= current_map_.info.height) {
            return false;
        }
        int index = cell.y * current_map_.info.width + cell.x;
        return current_map_.data[index] < 50; // Assuming < 50 means free space
    }

    // Calculate Manhattan distance heuristic
    double calculateHeuristic(const CellIndex& start, const CellIndex& goal) {
        return std::abs(goal.x - start.x) + std::abs(goal.y - start.y);
    }

    // Get movement cost between adjacent cells
    double getMovementCost(const CellIndex& from, const CellIndex& to) {
        int dx = std::abs(to.x - from.x);
        int dy = std::abs(to.y - from.y);
        return (dx + dy > 1) ? 1.414 : 1.0; // Diagonal movement costs sqrt(2)
    }

    // Convert world coordinates to grid coordinates
    CellIndex worldToGrid(const geometry_msgs::msg::Point& point) {
        int x = static_cast<int>((point.x - current_map_.info.origin.position.x) / 
                                current_map_.info.resolution);
        int y = static_cast<int>((point.y - current_map_.info.origin.position.y) / 
                                current_map_.info.resolution);
        return CellIndex(x, y);
    }

    // Convert grid coordinates to world coordinates
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

        int[] map_data = map_sub_.data;

        
        // getting covariance matrix
        float[36] robot_pose = odom_sub_.pose.covariance;
        int robot_x = int(robot_pose.x);
        int robot_y = int(robot_pose.y);
        // round position to nearest int and assign it a cell
        //cells are 0,299 x and y, and coordinates range too
        // to find correct cell, convert robot pose coords to a specific cell in map memory
        int x = (-15+15) * map_sub_.info.resolution;
        int y = (-15+15) * map_sub_.info.resolution;

        int robot_cell = y * width + x;

        //f-score of zero indicates its the start point?
        AStarNode start_node = new AStarNode(robot_cell,0);
        
        AStarNode goal_node = new AStarNode(goal_.point, 0);
        
        //how the final path will be stored as before publishing
        pose[] finalPath;

        //end flag
        bool pathFound = false;
        
        AStarNode current_node = start_node;
        vector<AStarNode> explored;
        vector<AStarNode> notExplored;

        explored.push_back(start_node);

        // main logic loop for path generation
        while (!pathFound){
          // finds all the neighbours and their f_scores
          lowestList = findLowestFScore(findNeighbours(current_node));
          // if only one node is the lowest, then use that
          if (lowestList.length() == 1){
            current_node = lowestList[0];
            lowestList_
          }
          
          explored.add(current_node);
          notExplored.remove(current_node);

          if (current_node == goal_node){
            pathFound = true;
          }
          // double check path hasn't already been found
          else if (!pathFound){
            // find all the neighbour nodes
            // MAKE THIS FUNCTION
            AStarNode[8] neighbours = findNeighbours(); 
            // loop through all neighbour nodes
            for (int i = 0; i < 8){
              //MAKE THIS FUNCTION(S)
              if (isTraversable(neighbours[i]) == true || hasExploredNode(neighbours[i])){
                // do yo thang
                // if this new path is shorter or this node hasn't been discovered before
                if (new path is shorter? || !hasExploredNode(neighbours[i]))
                // calculate f score relative to current?
                // ADD THIS METHOD TO NODE CLASS
                neighbours[i].setFScore();
                //what is the "parent in this case?"
                //ADD THIS ATTRIBUTE TO NODE CLASS
                current = neighbours[i].parent;
                if (hasExplored(neighbours[i]))
                {
                  // what is the point of this?
                  notExplored.add(neighbours[i])
                }
              }
              // if isn't traversable, skip
            }
          }
        }

        // assign final path poses to path message
        path = finalPath;
        // publishes an array of poses
        path_pub_->publish(path);
    }
};

// function implementations
// Returns list of nodes with the lowest f score (list in case multiple nodes have the same low f score)
vector<AStarNode> findLowestFScore(&vector<AStarNode> list){
  int lowestScore = INT_MAX;
  vector<AStarNode> lowestList;
  for (int i = 0; i > list.length(), i++){
    // Case 1: new low
    if (list[i].fscore < lowestScore)
    {
      lowestScore = list[i].fscore;
      // clear the current in the list since there's a new low
      lowestList.clear();
      lowestList.add(list[i];)
    }    
    // Case 2: multiple that are lowest
    if (list[i].fscore == lowestScore){
      lowestList.add(list[i]);
    }
  }
    return lowestList;
  }

  // creates list of neighbours with index ranging from top left to bottom right
vector<AStarNode> findNeighbours(AStarNode given_node){
  vector<AStarNode> neighbours;
  // get node cell index
  currentCell = given_node.cellidx;
  // array returns from top left to bottom right row by row
  // each row is 30 long, so left and right are -1 and +1, top and bottom are -30 and +30 assuming map[0] is bottom left
  
  // im too lazy to make a loop for this
  // this adds to end of list
  // if the final cell index is a negative number, skip indexing that b/c it doesn't exist
  // top row (left to right)
  for (int i = -31; i >= -29; i++){
    if (current_cell + i > -1){
      neighbours.push_back(new AStarNode(current_cell + i, 0);    
    }
  }
  // middle row
  for (int i = -1; i =< 1; i++;){
    // also skips the current cell you're on from being indexed as a neighbour
    if ((current_cell + i > -1) && i != 0){
      neighbours.push_back(new AStarNode(current_cell + i, 0); 
    }
  }
  // bottom row (left to right)
  for (int i = 29; i >= 31; i++){
    if (current_cell + i > -1){
      neighbours.push_back(new AStarNode(current_cell + i, 0);    
    }
  }

  return neighbours;
}






}



int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}