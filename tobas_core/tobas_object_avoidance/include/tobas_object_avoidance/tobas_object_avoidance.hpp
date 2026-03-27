#pragma once

#include <octomap/OcTree.h>
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <tobas_msgs/msg/repulsive_acceleration.hpp>
#include <tobas_msgs_adapter/odometry.hpp>
#include <tobas_msgs_adapter/repulsive_acceleration.hpp>
#include <tobas_node/node.hpp>

namespace tobas
{

class TobasObjectAvoidance : public BaseNode
{
  using self = TobasObjectAvoidance;
  using super = BaseNode;

public:
  explicit TobasObjectAvoidance(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  void octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg);
  void calculateRepulsiveForce();

  void configureParameters();

  ros2::SubscriberPtr<octomap_msgs::msg::Octomap> octomap_sub_;
  ros2::PublisherPtr<tobas_msgs::RepulsiveAcceleration> repulsive_acc_pub_;

  std::shared_ptr<octomap::OcTree> octree_;

  bool avoidance_enable_;
  double min_safety_distance_;
  double repulsive_gain_;
  double force_to_acc_gain_;
};

}  // namespace tobas
