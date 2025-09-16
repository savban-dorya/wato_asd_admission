#include "map_memory_node.hpp"
#include <chrono>
#include <memory>
#include <cmath>

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())) {

  // Initialize subscriber to costmap
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/costmap", 10, std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));

  // Initialize subscriber to odometry
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom/filtered", 10, std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));

  // Initialize timer to check distances
  timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&MapMemoryNode::updateOccupancyGrid, this));

  RCLCPP_INFO(this->get_logger(), "Map Memory Node has been initialized.");

  last_x_ = GRID_SIZE;
  last_y_ = GRID_SIZE;
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  // Store the received costmap as a pointer
  last_costmap_ = *msg;

  // Set costmap recieved to true
  costmap_received_ = true;

  RCLCPP_INFO(this->get_logger(), "Received new costmap.");
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  // Store the received odometry as a pointer
  odom_ = *msg;
  RCLCPP_INFO(this->get_logger(), "Received new odometry.");

  // Initialize initial position if not set
  if(last_x_ == GRID_SIZE && last_y_ == GRID_SIZE) {
    last_x_ = odom_.pose.pose.position.x;
    last_y_ = odom_.pose.pose.position.y;
    return;
  }

  double current_x = odom_.pose.pose.position.x;
  double current_y = odom_.pose.pose.position.y;

  double distance_moved = std::sqrt(std::pow(current_x - last_x_, 2) + std::pow(current_y - last_y_, 2));

  if(distance_moved >= DISTANCE_THRESHOLD) {
    if (!last_costmap_.data.empty()) {

      // Reset initial position to check distance from
      last_x_ = current_x;
      last_y_ = current_y;

      // Call function to update costmap
      updateOccupancyGrid();

    } else {

      RCLCPP_WARN(this->get_logger(), "Costmap data is empty, cannot update map memory.");
    }
  }

}

void MapMemoryNode::updateOccupancyGrid() {
  // Convert Costmap to global OccupancyGrid
  if(costmap_received_ && costmap_behind_){

    // Update costmap to the global OccupancyGrid
    
    // Get the yaw and current x
    double robot_x = odom_.pose.pose.position.x;
    double robot_y = odom_.pose.pose.position.y;

    double current_yaw, roll_placeholder, pitch_placeholder;
      tf2::Quaternion q(
      odom_.pose.pose.orientation.x,
      odom_.pose.pose.orientation.y,
      odom_.pose.pose.orientation.z,
      odom_.pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    m.getRPY(roll_placeholder, pitch_placeholder, current_yaw);
    
    // Loop through costmap
    for(int x = -last_costmap_.info.width/2; x < last_costmap_.info.width/2; x++) {
      for(int y = -last_costmap_.info.width/2; y < last_costmap_.info.height/2; y++) {
        
        // Convert to world coordinates
        double world_x = robot_x + 1;
        double world_y = robot_y + 1;

  
      }
    }



  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
