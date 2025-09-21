#ifndef CONTROL_NODE_HPP_
#define CONTROL_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "control_core.hpp"

class ControlNode : public rclcpp::Node {
  public:
    ControlNode();

  private:
    robot::ControlCore control_;

    // ROS2 Subscriptions and Publishers
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_velocity_pub_;

    // Timer
    rclcpp::TimerBase::SharedPtr control_timer_;

    // Functions
    double getYaw();
    double getDist(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b);
    void controlLoop();
    geometry_msgs::msg::PoseStamped findLookaheadPoint();
    geometry_msgs::msg::Twist computeVelocity(const geometry_msgs::msg::PoseStamped &target);

    // Data
    nav_msgs::msg::Path::SharedPtr current_path_;
    nav_msgs::msg::Odometry::SharedPtr robot_odom_;

    // Constants
    const double LOOKAHEAD_DISTANCE = 3.0; // m
    const double LOOKAHEAD_TOLERANCE = 0.05; // m
    const double GOAL_TOLERANCE = 1.0; // m
    const double LINEAR_SPEED = 1.0; // m/s
    const double ANGULAR_SPEED_LIMIT = 2.0;
    const double GRID_MAX = 15.0;
    const double GRID_MIN = -15.0;

    // Bools
    bool odom_received;
    bool path_received;
    bool goal_reached;

};

#endif
