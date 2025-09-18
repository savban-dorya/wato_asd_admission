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
  
  // Initialize publisher for OccupancyGrid
  occupancy_grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map_memory", 10);

  // Initialize timer to check distances
  timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&MapMemoryNode::updateOccupancyGrid, this));

  RCLCPP_INFO(this->get_logger(), "Map Memory Node has been initialized.");

  // Setting the last_x
  last_x_ = GRID_SIZE;
  last_y_ = GRID_SIZE;

  // Initialize booleans
  costmap_received_ = false;
  odom_recieved_ = false;
  map_out_of_date_ = true;

  global_occupancy_grid_.data.assign(GRID_SIZE * RESOLUTION * GRID_SIZE * RESOLUTION, 0);
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
  odom_recieved_ = true;

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
    if (costmap_received_ == true) {

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

double MapMemoryNode::getYaw(double w, double x, double y, double z)
{
    return std::atan2(2.0 * (w * z + x * y),
                      1.0 - 2.0 * (y * y + z * z));
}

void MapMemoryNode::updateOccupancyGrid() {
  // Convert Costmap to global OccupancyGrid
  if(costmap_received_ && odom_recieved_ && map_out_of_date_){

    // Update costmap to the global OccupancyGrid
    
    // Get the yaw and current x
    int robot_x = static_cast<int>(odom_.pose.pose.position.x * RESOLUTION);
    int robot_y = static_cast<int>(odom_.pose.pose.position.y * RESOLUTION);

    double yaw = getYaw(
      odom_.pose.pose.orientation.w, 
      odom_.pose.pose.orientation.x, 
      odom_.pose.pose.orientation.y, 
      odom_.pose.pose.orientation.z
    );

    RCLCPP_INFO(this->get_logger(), "Robot position in grid x: %d y: %d yaw RAD: %f yaw DEG: %f", robot_x, robot_y, yaw, yaw*180/3.14);

    // Loop through costmap
    for(int x_costmap = -static_cast<int>(last_costmap_.info.width/2); x_costmap < static_cast<int>(last_costmap_.info.width/2); x_costmap++) {
      for(int y_costmap = -static_cast<int>(last_costmap_.info.height/2); y_costmap < static_cast<int>(last_costmap_.info.height/2); y_costmap++) {
        
        // Account for rotation
        int world_x = robot_x + static_cast<int>(x_costmap * std::cos(yaw)) 
                              + static_cast<int>(y_costmap * std::sin(yaw));
                
        int world_y = robot_y - static_cast<int>(x_costmap * std::sin(yaw)) 
                              + static_cast<int>(y_costmap * std::cos(yaw));
        
        // Account for OccupancyGrid origin is represented as (width/2, height/2)
        world_x += (GRID_SIZE*RESOLUTION) / 2.0;
        world_y += (GRID_SIZE*RESOLUTION) / 2.0;

        // Convert to global OccupancyGrid
        if(world_x < GRID_SIZE * RESOLUTION && world_x >= 0 &&
           world_y < GRID_SIZE * RESOLUTION && world_y >= 0) {
          
          // Variable to store value of costmap at point
          int costmap_value_ = last_costmap_.data[y_costmap * GRID_SIZE * RESOLUTION + x_costmap];
          
          // Only replace if new costmap has higher chance of obstacle
          if (global_occupancy_grid_.data[world_y * GRID_SIZE * RESOLUTION + world_x] < costmap_value_);
          {
            global_occupancy_grid_.data[world_y * GRID_SIZE * RESOLUTION + world_x] = costmap_value_;
          }
        }
      }
    }

    publishMapMemory();
  } else if (!costmap_received_) {
    RCLCPP_WARN(this->get_logger(), "Costmap has not been received yet.");
  } else if (!map_out_of_date_) {
    RCLCPP_INFO(this->get_logger(), "Costmap is not behind the robot, skipping update.");
  } else if (!odom_recieved_) {
    RCLCPP_INFO(this->get_logger(), "No odometry to update OccupancyGrid");
  }
}

void MapMemoryNode::publishMapMemory(){
  global_occupancy_grid_.header.stamp = this->now();
  global_occupancy_grid_.header.frame_id = "sim_world";
  global_occupancy_grid_.info.resolution = 1.0 / RESOLUTION; // meters per cell
  global_occupancy_grid_.info.width = GRID_SIZE * RESOLUTION;
  global_occupancy_grid_.info.height = GRID_SIZE * RESOLUTION;
  global_occupancy_grid_.info.origin.position.x = -GRID_SIZE / 2.0; // Center the grid
  global_occupancy_grid_.info.origin.position.y = -GRID_SIZE / 2.0;
  global_occupancy_grid_.info.origin.position.z = 0.0;
  global_occupancy_grid_.info.origin.orientation.w = 1.0;

  // Publish the global occupancy grid
  occupancy_grid_pub_->publish(global_occupancy_grid_);
  RCLCPP_INFO(this->get_logger(), "Published global occupancy grid.");

}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
