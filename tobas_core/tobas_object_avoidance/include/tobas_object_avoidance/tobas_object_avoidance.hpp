#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <tobas_msgs/msg/odometry.hpp>
#include <tobas_msgs/msg/repulsive_acceleration.hpp>

namespace tobas
{

class TobasObjectAvoidance : public rclcpp::Node
{
public:
  TobasObjectAvoidance();

private:
  void octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg);
  void odomCallback(const tobas_msgs::msg::Odometry::SharedPtr msg);
  void calculateRepulsiveForce();

  void configureParameters();

  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
  rclcpp::Subscription<tobas_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<tobas_msgs::msg::RepulsiveAcceleration>::SharedPtr repulsive_acc_pub;

  std::shared_ptr<octomap::OcTree> octree_;
  tobas_msgs::msg::Odometry::SharedPtr current_odom_;

  double min_safety_distance_;
  double repulsive_gain_;
};

}  // namespace tobas
