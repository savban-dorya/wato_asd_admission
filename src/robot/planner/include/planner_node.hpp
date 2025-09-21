#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "planner_core.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

class PlannerNode : public rclcpp::Node {
  public:
    PlannerNode();

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
      
      void updateScores(double g, double h) {
        g_score = g;
        h_score = h;
        f_score = g + h;
      }
    };

    
    // Comparator for the priority queue (min-heap by f_score)
    struct CompareF
    {
      bool operator()(const AStarNode* a, const AStarNode* b)
      {
        // We want the node with the smallest f_score on top
        return a->f_score > b->f_score;
      }
    };

  private:
    robot::PlannerCore planner_;


    // Subscribers and publishers
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Store State
    enum class State { WAITING_FOR_GOAL, WAITING_FOR_ROBOT_TO_REACH_GOAL };
    State state_;

    // Data Storage
    nav_msgs::msg::OccupancyGrid current_map_;
    geometry_msgs::msg::PointStamped goal_;
    geometry_msgs::msg::Pose robot_pose_;

    // Booleans
    bool goal_received_;

    // Callback functions
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void timerCallback();

    // Helper functions
    bool goalReached();
    bool isTraversable(const CellIndex& cell);
    double calculateHeuristic(const CellIndex& start, const CellIndex& goal);
    double getMovementCost(const CellIndex& from, const CellIndex& to);
    CellIndex worldToGrid(const geometry_msgs::msg::Point& point);
    geometry_msgs::msg::PoseStamped gridToWorld(const CellIndex& cell);
    void planPath();
};

#endif 
