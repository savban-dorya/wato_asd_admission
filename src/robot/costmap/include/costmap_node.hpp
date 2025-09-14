#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_
 
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include "costmap_core.hpp"

const int GRID_SIZE = 30;  // Grid size in meters 30 x 30
const int RESOLUTION = 10; // 1/resolution = number of cells per meter

class CostmapNode : public rclcpp::Node {
  public:
    CostmapNode();
    
    // Place callback function here
    void publishMessage();
 
  private:
    robot::CostmapCore costmap_;
    // Place these constructs here
    // rclcpp::Publisher<std_msgs::msg::String>::SharedPtr string_pub_;
    // rclcpp::TimerBase::SharedPtr timer_;

    void initializeCostmap();
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
    void inflateObstacles();
    void publishCostmap();

    //constants
    const int MAX_COST = 100; // Maximum cost value for occupied cells
    const int INFLATION_RADIUS = 1; // Number of cells around an obstacle to inflate

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_; // Subscription for LIDAR data
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_; // Publisher for the costmap
    int OccupancyGrid[GRID_SIZE*RESOLUTION][GRID_SIZE*RESOLUTION]; // Example size, adjust as needed
};
 
#endif 