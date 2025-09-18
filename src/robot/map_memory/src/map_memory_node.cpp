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
  last_x_ = LAST_XY_INIT;
  last_y_ = LAST_XY_INIT;

  // Initialize booleans
  costmap_received_ = false;
  map_out_of_date_ = true;

  global_occupancy_grid_.data.assign(ARRAY_SIZE * ARRAY_SIZE, 0);
}

// Store new recieved costmap and use the bool to show it is recieved
void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  // Store the received costmap as a pointer
  last_costmap_ = *msg;

  // Set costmap recieved to true
  costmap_received_ = true;

  RCLCPP_INFO(this->get_logger(), "Received new costmap.");
}

// Recieve odometry and check if the distance traveled calls for an update
void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  // Store the received odometry as a pointer
  odom_ = *msg;

  RCLCPP_INFO(this->get_logger(), "Received new odometry.");

  // Initialize initial position if not set
  if(last_x_ == LAST_XY_INIT && last_y_ == LAST_XY_INIT) {
    last_x_ = odom_.pose.pose.position.x;
    last_y_ = odom_.pose.pose.position.y;
    return;
  }

  // Store the current position and check how far it is from the intial position
  double current_x = odom_.pose.pose.position.x;
  double current_y = odom_.pose.pose.position.y;

  double distance_moved = std::sqrt(std::pow(current_x - last_x_, 2) + std::pow(current_y - last_y_, 2));

  // If its too far from initial position
  if(distance_moved >= DISTANCE_THRESHOLD && costmap_received_ == true) {
    

    // Reset initial position to check distance from
    last_x_ = current_x;
    last_y_ = current_y;

    // Shows the map is out of date will update next timer
    map_out_of_date_ = true;

    RCLCPP_INFO(this->get_logger(), "Distance threshold reached map is now out of date.");

  }
}


// Function to convert quaternion to Yaw
double MapMemoryNode::getYaw(double w, double x, double y, double z)
{
    return std::atan2(2.0 * (w * z + x * y),
                      1.0 - 2.0 * (y * y + z * z));
}

void MapMemoryNode::updateOccupancyGrid() {
  // Convert Costmap to global OccupancyGrid
  if(costmap_received_ && map_out_of_date_){

    // Update costmap to the global OccupancyGrid
    
    // Get the yaw and current position of the robot
    int robot_x = static_cast<int>(odom_.pose.pose.position.x * RESOLUTION);
    int robot_y = static_cast<int>(odom_.pose.pose.position.y * RESOLUTION);

    double yaw = getYaw(
      odom_.pose.pose.orientation.w, 
      odom_.pose.pose.orientation.x, 
      odom_.pose.pose.orientation.y, 
      odom_.pose.pose.orientation.z
    );

    // Loop through costmap
    for(int x_costmap = -static_cast<int>(last_costmap_.info.width/2); x_costmap < static_cast<int>(last_costmap_.info.width/2); x_costmap++) {
      for(int y_costmap = -static_cast<int>(last_costmap_.info.height/2); y_costmap < static_cast<int>(last_costmap_.info.height/2); y_costmap++) {
        
        // Account for rotation
        int world_x = robot_x + static_cast<int>(x_costmap * std::cos(yaw)) 
                              + static_cast<int>(y_costmap * std::sin(yaw));
                
        int world_y = robot_y - static_cast<int>(x_costmap * std::sin(yaw)) 
                              + static_cast<int>(y_costmap * std::cos(yaw));
        
        // Account for OccupancyGrid origin is represented as (width/2, height/2)
        world_x += (ARRAY_SIZE) / 2.0;
        world_y += (ARRAY_SIZE) / 2.0;

        // Convert to global OccupancyGrid
        if(world_x < ARRAY_SIZE && world_x >= 0 &&
           world_y < ARRAY_SIZE && world_y >= 0) {
          
          // Variable to store value of costmap at point
          int costmap_value_ = last_costmap_.data[y_costmap * GRID_SIZE + x_costmap];
          
          /*
          if(costmap_value_ == MAX_VALUE){
            RCLCPP_INFO(this->get_logger(), 
                        "Obstacle at | world_x:%d world_y:%d | x_costmap:%d y_costmap: %d Yaw: %f", 
                        world_x, world_y, x_costmap + 150, y_costmap + 150, yaw);
          }
          */

          // Only replace if new costmap has higher chance of obstacle
          if (global_occupancy_grid_.data[world_y * GRID_SIZE + world_x] < costmap_value_);
          {
            global_occupancy_grid_.data[world_y * GRID_SIZE  + world_x] = costmap_value_;
          }
        }
      }
    }

    // Publish new grid and show that the map is now up to date
    publishMapMemory();
    map_out_of_date_ = false;

  } else if (!costmap_received_) {
    RCLCPP_WARN(this->get_logger(), "Costmap has not been received yet.");
  } else if (!map_out_of_date_) {
    RCLCPP_INFO(this->get_logger(), "Costmap is not behind the robot, skipping update.");
  }
}

void MapMemoryNode::publishMapMemory(){
  // Set info for the OccupancyGrid

  global_occupancy_grid_.header.stamp = this->now();
  global_occupancy_grid_.header.frame_id = "sim_world";
  global_occupancy_grid_.info.resolution = 1.0 / RESOLUTION; // meters per cell
  global_occupancy_grid_.info.width = ARRAY_SIZE;
  global_occupancy_grid_.info.height = ARRAY_SIZE;
  global_occupancy_grid_.info.origin.position.x = -GRID_SIZE / 2.0; // Center the grid keep in mind it does not have the resolution
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
