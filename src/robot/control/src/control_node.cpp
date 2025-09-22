#include "control_node.hpp"
#include <cmath>


ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore(this->get_logger())) {
  // Subscribers and Publishers
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
              "/path", 10, [this](const nav_msgs::msg::Path::SharedPtr msg) { 
                // Store msg and only flip boolean if non empty
                current_path_ = msg; 
                if (!current_path_->poses.empty()) {
                  path_received = true;
                }
              });

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
              "/odom/filtered", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                // Store odometry and set odom_received to true
                robot_odom_ = msg; 
                odom_received = true; 
              });

  cmd_velocity_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  // Timer
  control_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), [this]() { 
                     controlLoop(); 
                   });

  // Initialize bools
  odom_received = false;
  path_received = false;
  goal_reached = false;
}




// Function to convert quaternion to Yaw
double ControlNode::getYaw()
{
  double w = robot_odom_->pose.pose.orientation.w;
  double x = robot_odom_->pose.pose.orientation.x;
  double y = robot_odom_->pose.pose.orientation.y;
  double z = robot_odom_->pose.pose.orientation.z;

  return std::atan2(2.0 * (w * z + x * y),
                    1.0 - 2.0 * (y * y + z * z));
}


// Calculates distance rounded to nearest tenth
double ControlNode::getDist(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b)
{
  return sqrt(std::pow(b.x - a.x , 2) + std::pow(b.y - a.y, 2));
}



geometry_msgs::msg::PoseStamped ControlNode::findLookaheadPoint()
{
  // Store current position
  geometry_msgs::msg::Point robot_position = robot_odom_->pose.pose.position;


  // If within lookahead distance to goal set goal as the next lookahead distance
  if(getDist(robot_position, current_path_->poses.back().pose.position) <= LOOKAHEAD_DISTANCE)
  {
    return current_path_->poses.back();
  }

  // Use odometry and path to find look ahead point checks every element before the last (goal) 
  for(int i = 0; i < current_path_->poses.size() - 1; i++)
  {

    double distance_ = getDist(robot_position, current_path_->poses[i].pose.position);

    // Check if it meets the look ahead distance
    if(distance_ >= LOOKAHEAD_DISTANCE - 0.05 &&
       distance_ <= LOOKAHEAD_DISTANCE + 0.05)
    {
      return current_path_->poses[i];
    }
  }

  // Returns the goal as a look ahead point
  return current_path_->poses.back();
}

geometry_msgs::msg::Twist ControlNode::computeVelocity(const geometry_msgs::msg::PoseStamped &target)
{
  // Compute velocity
  geometry_msgs::msg::Twist command_velocity_;
  double distance_ = getDist(robot_odom_->pose.pose.position, current_path_->poses.back().pose.position);

  // Calculate angular velocity
  double dx = target.pose.position.x - robot_odom_->pose.pose.position.x;
  double dy = target.pose.position.y - robot_odom_->pose.pose.position.y;
  double target_angle = std::atan2(dy, dx);  // [-pi, pi]
  
  // Yaw is the robots current orientation
  double yaw = getYaw();

  double angle_correction = target_angle - yaw;

  // Adjust for direction whichever is easier ensures its within (pi, -pi)
  while (angle_correction > M_PI){
    angle_correction -= 2.0 * M_PI;
  }
  while (angle_correction < -M_PI) {
    angle_correction += 2.0 * M_PI;
  }

  // Set angular velocity but dont let it be too fast
  command_velocity_.angular.z = std::clamp(angle_correction, -ANGULAR_SPEED_LIMIT, ANGULAR_SPEED_LIMIT); 

  // Check if point isn't goal 
  if(target != current_path_->poses.back())
  {
    // Set linear speed
    command_velocity_.linear.x = LINEAR_SPEED;

  } else {
    // Set linear speed so it slows down towards the goal if greater than the linear speed take the linear speed
    double formula_speed_ = (LINEAR_SPEED / LOOKAHEAD_DISTANCE) * distance_;
    command_velocity_.linear.x = std::min(formula_speed_, LINEAR_SPEED);
    RCLCPP_INFO(this->get_logger(), "Lookahead point is the goal setting speed to %f" , command_velocity_.linear.x);
  }

  return command_velocity_;
}





void ControlNode::controlLoop()
{
  // Only try if odometry and path is received
  if(odom_received && path_received)
  {
    // Find lookahead point
    geometry_msgs::msg::PoseStamped look_ahead_point_ = findLookaheadPoint();

    // Get command velocity
    geometry_msgs::msg::Twist command_velocity_ = computeVelocity(look_ahead_point_);

    // Publish it (controls the robot) if not within goal tolerance yet
    if(getDist(robot_odom_->pose.pose.position, current_path_->poses.back().pose.position) > GOAL_TOLERANCE){

      RCLCPP_INFO(this->get_logger(), "Command Velocity Published: %f with distance from goal: %f", command_velocity_.linear.x, getDist(robot_odom_->pose.pose.position, current_path_->poses.back().pose.position) );
      cmd_velocity_pub_->publish(command_velocity_);

    } else
    {
      // Publish zero as the velocity
      command_velocity_.linear.x = 0.0;
      command_velocity_.angular.z = 0.0;
      cmd_velocity_pub_->publish(command_velocity_);

      RCLCPP_INFO(this->get_logger(), "Goal has been reached");
    }
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
