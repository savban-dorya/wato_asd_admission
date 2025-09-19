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
  map_out_of_date_ = false;

  // Initialize map to 0 (empty)
  global_occupancy_grid_.data.assign(ARRAY_WIDTH * ARRAY_HEIGHT, 0);
}




// Store new recieved costmap and use the bool to show it is recieved
void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  // Store the received costmap as a pointer
  last_costmap_ = *msg;

  // Set costmap recieved to true
  costmap_received_ = true;

  RCLCPP_INFO(this->get_logger(), "Costmap Recieved");
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
    map_out_of_date_ = true;
    return;
  }

  // Store the current position and check how far it is from the intial position
  double current_x = odom_.pose.pose.position.x;
  double current_y = odom_.pose.pose.position.y;

  double distance_moved = std::sqrt(std::pow(current_x - last_x_, 2) + std::pow(current_y - last_y_, 2));

  // If its too far from initial position
  if(distance_moved >= DISTANCE_THRESHOLD) {
    

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



int MapMemoryNode::getIndex(int x, int y){
  return (y + POINT_TO_ARRAY) * ARRAY_WIDTH + (x + POINT_TO_ARRAY);
}






void MapMemoryNode::updateOccupancyGrid() {
  // Convert Costmap to global OccupancyGrid
  RCLCPP_INFO(this->get_logger(), "updateOccupancy Grid Called");


  if(costmap_received_ && map_out_of_date_){

    // Update costmap to the global OccupancyGrid

    // Get Robot pose
    double robot_to_origin_x = odom_.pose.pose.position.x * RESOLUTION;
    double robot_to_origin_y = odom_.pose.pose.position.y * RESOLUTION;

    double robot_yaw = getYaw(
      odom_.pose.pose.orientation.w, 
      odom_.pose.pose.orientation.x, 
      odom_.pose.pose.orientation.y, 
      odom_.pose.pose.orientation.z
    );

    // Loop through every point in costmap and convert it to the grid
    for(int costmap_point_x = -POINT_TO_ARRAY; costmap_point_x < POINT_TO_ARRAY; costmap_point_x++)
    {
      for(int costmap_point_y = -POINT_TO_ARRAY; costmap_point_y < POINT_TO_ARRAY; costmap_point_y++)
      {
          // Account for rotation
          double unrotated_costmap_point_x = costmap_point_x * std::cos(robot_yaw)  - costmap_point_y * std::sin(robot_yaw);

          double unrotated_costmap_point_y = costmap_point_x * std::sin(robot_yaw) + costmap_point_y * std::cos(robot_yaw);

          // Find point in the world using (0,0) as the center of the world
          int world_to_origin_x = static_cast<int>(robot_to_origin_x + unrotated_costmap_point_x);
          int world_to_origin_y = static_cast<int>(robot_to_origin_y + unrotated_costmap_point_y);  

          // Get Array Indices for the points
          int global_array_index = getIndex(world_to_origin_x, world_to_origin_y);
          int costmap_array_index = getIndex(costmap_point_x, costmap_point_y);

          // Check if within bounds
          if(global_array_index < ARRAY_WIDTH * ARRAY_HEIGHT && global_array_index >= 0)
          {
            if(global_occupancy_grid_.data[global_array_index] < last_costmap_.data[costmap_array_index])
            {
              global_occupancy_grid_.data[global_array_index] = last_costmap_.data[costmap_array_index];

              if(last_costmap_.data[costmap_array_index] > 0)
              {
                /*
                RCLCPP_INFO(this->get_logger(), "Obstacle at x: %d y: %d cval: %d occ_val: %d", 
                                                world_to_origin_x, world_to_origin_y, last_costmap_.data[costmap_array_index], 
                                                global_occupancy_grid_.data[global_array_index]);
                */
              }
            }
          }
      }
    }

    // Set back to false
    publishMapMemory();
    costmap_received_ = false;
    map_out_of_date_ = false;

    int occupied = 0;
    for (auto val : last_costmap_.data) {
        if (val > 0) occupied++;
    }

    RCLCPP_INFO(this->get_logger(), "Number of obstacles: %d", occupied);

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
  global_occupancy_grid_.info.width = ARRAY_WIDTH;
  global_occupancy_grid_.info.height = ARRAY_HEIGHT;
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
