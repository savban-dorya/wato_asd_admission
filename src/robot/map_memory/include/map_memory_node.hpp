#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "map_memory_core.hpp"

const int GRID_SIZE = 30;  // Grid size in meters 30 x 30
const int RESOLUTION = 10; // 1/resolution = number of cells per meter

class MapMemoryNode : public rclcpp::Node {
  public:
    MapMemoryNode();

  private:
    robot::MapMemoryCore map_memory_;
    nav_msgs::msg::OccupancyGrid global_occupancy_grid_;
    nav_msgs::msg::OccupancyGrid last_costmap_;
    nav_msgs::msg::Odometry odom_;

    //publisher
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_pub_;

    //subscriptions
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    //timer
    rclcpp::TimerBase::SharedPtr timer_;

    //functions
    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void updateOccupancyGrid();
    void publishMapMemory();
    double getYaw(double w, double x, double y, double z);

    //variables
    double last_x_, last_y_;

    // Bool to check if costmap has been recieved and if it should be updated
    bool costmap_received_ = false;
    bool costmap_behind_ = true;

    // Constants
    const double DISTANCE_THRESHOLD = 1.5; // meters


};

#endif 
