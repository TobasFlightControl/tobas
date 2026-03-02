#include "tobas_object_avoidance/tobas_object_avoidance.hpp"

#include <tobas_constants/constants.hpp>

namespace tobas
{

TobasObjectAvoidance::TobasObjectAvoidance() : Node("tobas_object_avoidance")
{
  min_safety_distance_ = this->declare_parameter("min_safety_distance", 30.0);
  repulsive_gain_ = this->declare_parameter("repulsive_gain", 1.0);

  octomap_sub_ = this->create_subscription<octomap_msgs::msg::Octomap>(
    "/object_octomap", 10, std::bind(&TobasObjectAvoidance::octomapCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<tobas_msgs::msg::Odometry>(
    tobas::kOdometryTopic, 10, std::bind(&TobasObjectAvoidance::odomCallback, this, std::placeholders::_1));

  repulsive_acc_pub = this->create_publisher<tobas_msgs::msg::RepulsiveAcceleration>("/repulsive_acc", 10);
}

void TobasObjectAvoidance::octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg)
{
  octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(*msg);
  if (tree) {
    octree_ = std::shared_ptr<octomap::OcTree>(dynamic_cast<octomap::OcTree*>(tree));
    if (!octree_) {
      RCLCPP_ERROR(this->get_logger(), "Failed to cast AbstractOcTree to OcTree");
      delete tree;
    }
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to deserialize OctoMap");
  }

  calculateRepulsiveForce();
}

void TobasObjectAvoidance::odomCallback(const tobas_msgs::msg::Odometry::SharedPtr msg)
{
  current_odom_ = msg;
}

void TobasObjectAvoidance::calculateRepulsiveForce()
{
  if (!octree_ || !current_odom_) return;

  double rho_0 = min_safety_distance_;

  double force_x = 0.0;
  double force_y = 0.0;
  double force_z = 0.0;

  for (auto it = octree_->begin_leafs(); it != octree_->end_leafs(); ++it) {
    if (octree_->isNodeOccupied(*it)) {
      double ox = it.getX();
      double oy = it.getY();
      double oz = it.getZ();

      double dist_sq = ox * ox + oy * oy + oz * oz;
      double dist = std::sqrt(dist_sq);

      if (dist < rho_0 && dist > 1e-3) {
        double mag = repulsive_gain_ * (1.0 / dist - 1.0 / rho_0) * (1.0 / dist_sq);
        
        force_x += mag * (-ox / dist);
        force_y += mag * (-oy / dist);
        force_z += mag * (-oz / dist);
      }
    }
  }

  tobas_msgs::msg::RepulsiveAcceleration repulsive_acc_msg;
  repulsive_acc_msg.header.stamp = this->get_clock()->now();
  repulsive_acc_msg.header.frame_id = "base_link";

  repulsive_acc_msg.accel.x = force_x;
  repulsive_acc_msg.accel.y = force_y;
  repulsive_acc_msg.accel.z = force_z;

  repulsive_acc_pub->publish(repulsive_acc_msg);
}

}  // namespace tobas

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<tobas::TobasObjectAvoidance>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
