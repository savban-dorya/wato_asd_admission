#include <chrono>
#include <memory>
#include <cmath>

 
#include "costmap_node.hpp"


CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  // Initialize the constructs and their parameters

  // Initialize subscription to LIDAR data
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar", 10, std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));

  // Initialize publisher for the costmap
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);

  initializeCostmap();

  // string_pub_ = this->create_publisher<std_msgs::msg::String>("/test_topic", 10);
  // timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&CostmapNode::publishMessage, this));
}

/*
// Define the timer to publish a message every 500ms
void CostmapNode::publishMessage() {
  auto message = std_msgs::msg::String();
  message.data = "Hello, ROS 2!";
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  string_pub_->publish(message);
}
*/

void CostmapNode::initializeCostmap() {
  // Initialize the occupancy grid to unknown (-1)
  for (int i = 0; i < GRID_SIZE * RESOLUTION; ++i) {
    for (int j = 0; j < GRID_SIZE * RESOLUTION; ++j) {
      OccupancyGrid[i][j] = -1; // Unknown
    }
  }
}

void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  // Process the incoming laser scan data and update the occupancy grid
  int num_readings = static_cast<int>((scan->angle_max - scan->angle_min) / scan->angle_increment);

  for (int i = 0; i < num_readings; i++) {
    // Get the distance reading from the ranges array
    float dist_reading = scan->ranges[i];
    if (dist_reading < scan->range_min || dist_reading > scan->range_max) {
      continue; // Ignore invalid readings
    }

    // angle of the current reading
    float angle = scan->angle_min + i * scan->angle_increment;

    // Convert polar coordinates (dist_reading, angle) to grid coordinates (x, y)
    // + (GRID_SIZE / 2.0) to center the robot in the grid so (0,0) is at the center of the grid
    int x = static_cast<int>((dist_reading * cos(angle) + (GRID_SIZE / 2.0)) * RESOLUTION);
    int y = static_cast<int>((dist_reading * sin(angle) + (GRID_SIZE / 2.0)) * RESOLUTION);

    // Mark occupied if within grid bounds
    if (x >= 0 && x < GRID_SIZE * RESOLUTION && y >= 0 && y < GRID_SIZE * RESOLUTION) {
      OccupancyGrid[x][y] = MAX_COST; // Mark as occupied
    }
  }

  inflateObstacles();
  publishCostmap();

  RCLCPP_INFO(this->get_logger(), "Laser scan processed and occupancy grid updated.");
}

void CostmapNode::inflateObstacles(){
  // 2 loops to traverse the grid looking for occupied cells (i,j)
  for (int i = 0; i < GRID_SIZE * RESOLUTION; ++i) {
    for (int j = 0; j < GRID_SIZE * RESOLUTION; ++j) {

      // If an occupied cell is found, inflate around it
      if (OccupancyGrid[i][j] == MAX_COST) {

        //traverse using dx and dy for distance from the occupied cell
        for (int dx = -INFLATION_RADIUS; dx <= INFLATION_RADIUS; ++dx) {
          for (int dy = -INFLATION_RADIUS; dy <= INFLATION_RADIUS; ++dy) {

            //convert  to grid coordinates
            int x = i + dx;
            int y = j + dy;

            // Check bounds and inflate using the formula
            if (x >= 0 && x < GRID_SIZE * RESOLUTION && y >= 0 && y < GRID_SIZE * RESOLUTION) {

              //check if already occupied
              if (OccupancyGrid[x][y] != MAX_COST) {
                OccupancyGrid[x][y] = MAX_COST * ((INFLATION_RADIUS - std::sqrt(dx * dx + dy * dy)) / INFLATION_RADIUS);
              }
            }
          }
        }
      }
    }
  }
}

void CostmapNode::publishCostmap() {
  // Convert the occupancy grid to a ROS2 message and publish it
  nav_msgs::msg::OccupancyGrid costmap_msg = nav_msgs::msg::OccupancyGrid();

  // Fill in the header and info
  costmap_msg.header.stamp = this->now();
  costmap_msg.header.frame_id = "map";
  costmap_msg.info.resolution = 1.0 / RESOLUTION; // meters per cell
  costmap_msg.info.width = GRID_SIZE * RESOLUTION;
  costmap_msg.info.height = GRID_SIZE * RESOLUTION;
  costmap_msg.info.origin.position.x = -GRID_SIZE / 2.0; // Center the grid
  costmap_msg.info.origin.position.y = -GRID_SIZE / 2.0;
  costmap_msg.info.origin.position.z = 0.0;
  costmap_msg.info.origin.orientation.w = 1.0;

  // Flatten the 2D occupancy grid into a 1D array for the message
  costmap_msg.data.resize(GRID_SIZE * RESOLUTION * GRID_SIZE * RESOLUTION);
  for (int i = 0; i < GRID_SIZE * RESOLUTION; ++i) {
    for (int j = 0; j < GRID_SIZE * RESOLUTION; ++j) {
      costmap_msg.data[i * GRID_SIZE * RESOLUTION + j] = OccupancyGrid[i][j];
    }
  }

  // Publish the costmap message
  costmap_pub_->publish(costmap_msg);
  RCLCPP_INFO(this->get_logger(), "Costmap published.");
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}