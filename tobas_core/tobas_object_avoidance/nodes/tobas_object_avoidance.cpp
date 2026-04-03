#include <octomap/OcTree.h>
#include <octomap/octomap.h>

#include <tobas_constants/ros_interface.hpp>
#include <tobas_node/node.hpp>

#include <octomap_msgs/conversions.h>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <octomap_msgs/msg/octomap.hpp>

#include <tobas_msgs/msg/repulsive_acceleration.hpp>
#include <tobas_msgs_adapter/odometry.hpp>
#include <tobas_msgs_adapter/repulsive_acceleration.hpp>

namespace tobas
{

class ObjectAvoidance : public BaseNode
{
  using self = ObjectAvoidance;
  using super = BaseNode;

public:
  explicit ObjectAvoidance(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  void octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg);
  void calculateRepulsiveForce();

  void configureParameters();

  ros2::SubscriberPtr<octomap_msgs::msg::Octomap> octomap_sub_;
  ros2::PublisherPtr<tobas_msgs::RepulsiveAcceleration> repulsive_acc_pub_;

  std::unique_ptr<octomap::OcTree> octree_;

  bool avoidance_enable_;
  double min_safety_distance_;
  double repulsive_gain_;
  double force_to_acc_gain_;
};

ObjectAvoidance::ObjectAvoidance(const rclcpp::NodeOptions& options)
  : super("object_avoidance", nodeOptions_Default(options))
{
  avoidance_enable_ = BaseNode::getBoolParam("avoidance_enable", true);
  min_safety_distance_ = BaseNode::getDoubleParam("min_safety_distance", 30.0);
  repulsive_gain_ = BaseNode::getDoubleParam("repulsive_gain", 1.0);
  force_to_acc_gain_ = BaseNode::getDoubleParam("force_to_acc_gain_", 1.0);

  octomap_sub_ = this->create_subscription<octomap_msgs::msg::Octomap>(
    "/object_octomap", 10, std::bind(&ObjectAvoidance::octomapCallback, this, std::placeholders::_1));

  repulsive_acc_pub_ = createPublisher<tobas_msgs::RepulsiveAcceleration>(topic::kRepulsiveAccel);
}

void ObjectAvoidance::octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg)
{
  if (!avoidance_enable_) {
    return;
  }
  octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(*msg);
  if (tree) {
    octree_.reset(dynamic_cast<octomap::OcTree*>(tree));
    if (!octree_) {
      TOBAS_ERROR("Failed to cast AbstractOcTree to OcTree");
      delete tree;
    }
  }
  else {
    TOBAS_ERROR("Failed to deserialize OctoMap");
  }

  calculateRepulsiveForce();
}

void ObjectAvoidance::calculateRepulsiveForce()
{
  if (!octree_) {
    return;
  }

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

      if (dist < min_safety_distance_ && dist > 1e-3) {
        double mag = repulsive_gain_ * (1.0 / dist - 1.0 / min_safety_distance_) * (1.0 / dist_sq);

        force_x += mag * (-ox / dist);
        force_y += mag * (-oy / dist);
        force_z += mag * (-oz / dist);
      }
    }
  }

  tobas_msgs::msg::RepulsiveAcceleration repulsive_acc_msg;
  repulsive_acc_msg.header.stamp = this->get_clock()->now();
  repulsive_acc_msg.header.frame_id = "base_link";

  repulsive_acc_msg.accel.x = force_x * force_to_acc_gain_;
  repulsive_acc_msg.accel.y = force_y * force_to_acc_gain_;
  repulsive_acc_msg.accel.z = force_z * force_to_acc_gain_;

  repulsive_acc_pub_->publish(repulsive_acc_msg);
}

}  // namespace tobas

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<tobas::ObjectAvoidance>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
