// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <tobas_constants/ros_interface.hpp>
#include <tobas_kdl/tree_fk_solver_pos_all.hpp>
#include <tobas_kdl/tree_inertia_solver.hpp>
#include <tobas_node/node.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_std_tools/universal_constants.hpp>
#include <tobas_tools/tree_joint_state_converter.hpp>

#include <tobas_drone_msgs_adapter/drone.hpp>
#include <tobas_kdl_msgs_adapter/tree.hpp>
#include <tobas_kdl_msgs_adapter/wrench_stamped.hpp>
#include <tobas_msgs/msg/joint_state_array.hpp>
#include <tobas_msgs/msg/rotor_liveliness_array.hpp>
#include <tobas_msgs/msg/rotor_state_array.hpp>
#include <tobas_msgs_adapter/odometry_with_covariance_stamped.hpp>

namespace tobas
{
class DisturbanceObserverNode : public BaseNode
{
  using self = DisturbanceObserverNode;
  using super = BaseNode;

public:
  explicit DisturbanceObserverNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  kdl::Tree tree_;
  Drone::ConstSharedPtr drone_;

  kdl::TreeFkSolverPosAll fk_solver_;
  kdl::TreeInertiaSolver inertia_solver_;
  TreeJointStateConverter js_converter_;

  std::map<std::string, double> rotor_thrusts_;
  bool js_received_ = false;

  ros2::PublisherPtr<tobas_kdl_msgs::WrenchStamped> dist_force_pub_;

  ros2::SubscriberPtr<kdl::Tree> tree_sub_;
  ros2::SubscriberPtr<Drone> drone_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::RotorStateArray> rotor_states_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::RotorLivelinessArray> rotor_liveliness_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::JointStateArray> joint_states_sub_;
  ros2::SubscriberPtr<tobas_msgs::OdometryWithCovarianceStamped> odom_sub_;

  void treeCb(const kdl::Tree::ConstSharedPtr& tree);
  void droneCb(const Drone::ConstSharedPtr& drone);
  void rotorStatesCb(const tobas_msgs::msg::RotorStateArray::ConstSharedPtr& rotor_states);
  void rotorLivelinessCb(const tobas_msgs::msg::RotorLivelinessArray::ConstSharedPtr& rotor_liveliness);
  void jointStatesCb(const tobas_msgs::msg::JointStateArray::ConstSharedPtr& joint_states);
  void odomCb(const tobas_msgs::OdometryWithCovarianceStamped::ConstSharedPtr& odom);
};

DisturbanceObserverNode::DisturbanceObserverNode(const rclcpp::NodeOptions& options)
  : super("disturbance_observer", nodeOptions_Default(options))
  , fk_solver_(tree_)
  , inertia_solver_(tree_)
  , js_converter_(tree_)
{
  dist_force_pub_ = createPublisher<tobas_kdl_msgs::WrenchStamped>(topic::kDisturbanceForce);

  tree_sub_ = createSubscriber(topic::kKdlTree, &self::treeCb, this, true, true);
  drone_sub_ = createSubscriber(topic::kDrone, &self::droneCb, this, true, true);
  rotor_states_sub_ = createSubscriber(topic::kRotorStates, &self::rotorStatesCb, this);
  rotor_liveliness_sub_ = createSubscriber(topic::kRotorLiv, &self::rotorLivelinessCb, this);
  odom_sub_ = createSubscriber(topic::kOdometry, &self::odomCb, this);
}

void DisturbanceObserverNode::treeCb(const kdl::Tree::ConstSharedPtr& tree)
{
  tree_ = *tree;

  fk_solver_.updateInternalDataStructures();
  inertia_solver_.updateInternalDataStructures();
  js_converter_.updateInternalDataStructures();
}

void DisturbanceObserverNode::droneCb(const Drone::ConstSharedPtr& drone)
{
  drone_ = drone;

  rotor_thrusts_.clear();
  js_received_ = false;

  if (drone->hasServoJoint()) {
    joint_states_sub_ = createSubscriber(topic::kJointStates, &self::jointStatesCb, this);
  }
  else {
    joint_states_sub_.reset();
  }
}

void DisturbanceObserverNode::rotorStatesCb(const tobas_msgs::msg::RotorStateArray::ConstSharedPtr& rotor_states)
{
  if (tree_.empty()) {
    return;
  }

  if (!drone_) {
    return;
  }

  for (const auto& elem : rotor_states->states) {
    const auto thrust_it = rotor_thrusts_.find(elem.link_name);
    if (thrust_it == rotor_thrusts_.end()) {
      // Add this rotor if the link name is included in the vehicle model.
      if (!tree_.hasSegment(elem.link_name) || !drone_->prop->rotors.contains(elem.link_name)) {
        TOBAS_ERROR("The drone does not have rotor named \"", elem.link_name, "\".");
        continue;
      }
      rotor_thrusts_[elem.link_name] = 0.0;
    }
    else {
      // Update thrust only when the rotor state is available.
      // In other words, assume the thrust has not changed if the rotor state is temporarily unavailable.
      if (elem.status == tobas_msgs::msg::RotorState::NO_ERROR) {
        thrust_it->second = elem.thrust;
      }
    }
  }
}

void DisturbanceObserverNode::rotorLivelinessCb(
  const tobas_msgs::msg::RotorLivelinessArray::ConstSharedPtr& rotor_liveliness)
{
  for (const auto& elem : rotor_liveliness->data) {
    const auto thrust_it = rotor_thrusts_.find(elem.link_name);
    if (thrust_it == rotor_thrusts_.end()) {
      continue;
    }

    // Treat the thrust as zero if the rotor is dead.
    if (!elem.alive) {
      thrust_it->second = 0.0;
    }
  }
}

void DisturbanceObserverNode::jointStatesCb(const tobas_msgs::msg::JointStateArray::ConstSharedPtr& joint_states)
{
  if (js_converter_.convert(*joint_states) < 0) {
    TOBAS_ERROR("Joint state converter failed: ", js_converter_.errorMessage());
    return;
  }

  js_received_ = true;
}

void DisturbanceObserverNode::odomCb(const tobas_msgs::OdometryWithCovarianceStamped::ConstSharedPtr& odom)
{
  if (tree_.empty()) {
    return;
  }

  if (!drone_) {
    return;
  }

  if (joint_states_sub_ && !js_received_) {
    return;
  }

  // Calculate forward kinematics.
  if (fk_solver_.jntToCart(js_converter_.getPosition()) < 0) {
    TOBAS_ERROR("Forward kinematics failed: ", fk_solver_.errorMessage());
    return;
  }

  // Calculate mass properties.
  if (inertia_solver_.jntToCart(js_converter_.getPosition()) < 0) {
    TOBAS_ERROR("Inertia solver failed: ", inertia_solver_.errorMessage());
    return;
  }
  const auto& inertia = inertia_solver_.getInertia();
  const auto B_Pos_B2G = inertia.getCOG();
  const auto I_B = inertia.getRotationalInertiaCoG();
  const auto& mass = inertia.getMass();
  const auto weight = mass * st::kGravity;

  // Calculate terms affected by thrust.
  kdl::Vector trans_sum = kdl::Vector::Zero();
  kdl::Vector rot_sum = kdl::Vector::Zero();
  for (const auto& [link_name, thrust] : rotor_thrusts_) {
    const auto& rotor = drone_->prop->rotors.at(link_name);

    const auto& elem = tree_.getSegment(link_name)->second;
    const auto& B_Rot_Par = fk_solver_.getFrame(elem.parent->first).M;
    const auto axis_B = B_Rot_Par * elem.segment.joint().axis();

    const auto d = rotor->sign();
    const auto cm = rotor->momentConst();
    const auto& B_Pos_B2P = fk_solver_.getFrame(link_name).p;
    const auto B_Pos_G2P = B_Pos_B2P - B_Pos_B2G;

    trans_sum += axis_B * thrust;
    rot_sum += (B_Pos_G2P * axis_B - (d * cm) * axis_B) * thrust;
  }

  // Calculate external force.
  // TODO: Also consider fixed-wing forces.
  // FIXME: DGyro numerical errors reduce external torque accuracy.
  const auto& W_Rot_B = odom->odom.odom.frame.M;
  const auto& gyro_B = odom->odom.odom.twist.rot;
  const auto& acc_B = odom->odom.odom.accel.linear;
  const auto& dgyro_B = odom->odom.odom.accel.angular;
  const auto force_W = kdl::Vector(0, 0, weight) + W_Rot_B * (mass * acc_B - trans_sum);
  const auto torque_B = I_B * dgyro_B + gyro_B * (I_B * gyro_B) - rot_sum;

  // Create external force message.
  auto dist_force_msg = std::make_unique<tobas_kdl_msgs::WrenchStamped>();
  dist_force_msg->header.stamp = odom->header.stamp;
  dist_force_msg->wrench.force = force_W;
  dist_force_msg->wrench.torque = torque_B;

  // Publish external force message.
  dist_force_pub_->publish(std::move(dist_force_msg));
}
}  // namespace tobas

RCLCPP_COMPONENTS_REGISTER_NODE(tobas::DisturbanceObserverNode)
