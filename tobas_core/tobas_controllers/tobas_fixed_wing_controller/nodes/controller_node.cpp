// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <ranges>

#include <tobas_constants/node.hpp>
#include <tobas_constants/throttle.hpp>
#include <tobas_constants/time.hpp>
#include <tobas_control/lqd.hpp>
#include <tobas_eigen_tools/core.hpp>
#include <tobas_kdl/jntarray.hpp>
#include <tobas_kdl/tree_mass_holder.hpp>
#include <tobas_node/node.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_std_tools/standard_atmosphere.hpp>
#include <tobas_std_tools/universal_constants.hpp>
#include <tobas_tools/command_priority_handler.hpp>
#include <tobas_tools/coordinates.hpp>
#include <tobas_tools/fixed_wing.hpp>

#include <tobas_command_msgs/msg/elev_aile_rud_throttle.hpp>
#include <tobas_drone_msgs_adapter/drone.hpp>
#include <tobas_kdl_msgs_adapter/tree.hpp>
#include <tobas_msgs/msg/arming.hpp>
#include <tobas_msgs/msg/fluid_pressure.hpp>
#include <tobas_msgs/msg/joint_command_array.hpp>
#include <tobas_msgs/msg/rotor_thrust_array.hpp>
#include <tobas_msgs_adapter/odometry_with_covariance_stamped.hpp>


namespace tobas
{
namespace fixed_wing
{
class ControllerNode : public BaseNode
{
  using self = ControllerNode;
  using super = BaseNode;

public:
  explicit ControllerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  Drone drone_;
  kdl::Tree tree_;

  kdl::TreeMassHolder mass_holder_;

  // State
  bool is_initialized_ = false;
  bool drone_received_ = false;
  bool tree_received_ = false;
  bool topics_received_ = false;
  CommandPriorityHandler cmd_priority_handler_;
  tobas_msgs::msg::FluidPressure::ConstSharedPtr air_pressure_;           // Atmospheric pressure
  tobas_msgs::OdometryWithCovarianceStamped::ConstSharedPtr odom_flu_;    // Current state in the FLU coordinate system
  tobas_msgs::msg::Arming::ConstSharedPtr arming_;                        // Rotor arming state

  // Command
  std::unique_ptr<Eigen::VectorXd> thrusts_ = nullptr;
  std::unique_ptr<Eigen::VectorXd> deflections_ = nullptr;

  // Publishers
  ros2::PublisherPtr<tobas_msgs::msg::RotorThrustArray> tar_thrusts_pub_;
  ros2::PublisherPtr<tobas_msgs::msg::JointCommandArray> tar_angles_pub_;

  // Subscribers
  ros2::SubscriberPtr<Drone> drone_sub_;
  ros2::SubscriberPtr<kdl::Tree> tree_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::FluidPressure> air_pressure_sub_;
  ros2::SubscriberPtr<tobas_msgs::OdometryWithCovarianceStamped> odom_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::Arming> arming_sub_;
  ros2::SubscriberPtr<tobas_command_msgs::msg::ElevAileRudThrottle> manual_cmd_sub_;

  // Timers
  ros2::TimerPtr check_topics_timer_;

  bool initialize();
  void publishThrusts(const builtin_interfaces::msg::Time& stamp, const Eigen::VectorXd& thrusts);
  void publishDeflections(const builtin_interfaces::msg::Time& stamp, const Eigen::VectorXd& deflections);
  bool isCommandAccepted(const tobas_command_msgs::msg::Priority& priority);

  void droneCb(const Drone::ConstSharedPtr& drone);
  void treeCb(const kdl::Tree::ConstSharedPtr& tree);
  void armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming);
  void airPressureCb(const tobas_msgs::msg::FluidPressure::ConstSharedPtr& pressure);
  void odomCb(const tobas_msgs::OdometryWithCovarianceStamped::ConstSharedPtr& odom_flu);
  void manualCmdCb(const tobas_command_msgs::msg::ElevAileRudThrottle::ConstSharedPtr& cmd);

  void checkTopicsTimerCb();
};

ControllerNode::ControllerNode(const rclcpp::NodeOptions& options)
  : super(node::kController, nodeOptions_DParam(options)), mass_holder_(tree_)
{
  // Register publishers.
  tar_thrusts_pub_ = createPublisher<tobas_msgs::msg::RotorThrustArray>(topic::kRotorThrustsCmd);
  tar_angles_pub_ = createPublisher<tobas_msgs::msg::JointCommandArray>(topic::kJointPosCmd);

  // Register subscribers.
  drone_sub_ = createSubscriber(topic::kDrone, &self::droneCb, this, true, true);
  tree_sub_ = createSubscriber(topic::kKdlTree, &self::treeCb, this, true, true);
  arming_sub_ = createSubscriber(topic::kArming, &self::armingCb, this);
  air_pressure_sub_ = createSubscriber(topic::kAirPressure, &self::airPressureCb, this);
  odom_sub_ = createSubscriber(topic::kOdometry, &self::odomCb, this);
  manual_cmd_sub_ = createSubscriber(topic::kElevAileRudThrottleCmd, &self::manualCmdCb, this);

  // Register timers.
  check_topics_timer_ = createTimer(kCheckTopicsPeriod, &self::checkTopicsTimerCb, this);
}

bool ControllerNode::initialize()
{
  if (!mass_holder_.updateInternalDataStructures()) {
    return false;
  }

  is_initialized_ = true;
  return true;
}

void ControllerNode::publishThrusts(const builtin_interfaces::msg::Time& stamp, const Eigen::VectorXd& thrusts)
{
  assert(static_cast<size_t>(thrusts.size()) == drone_.prop->numRotors());

  auto thrusts_msg = std::make_unique<tobas_msgs::msg::RotorThrustArray>();
  thrusts_msg->header.stamp = stamp;

  for (const auto& [idx, elem] : std::views::enumerate(drone_.prop->rotors)) {
    thrusts_msg->thrusts.emplace_back();
    thrusts_msg->thrusts.back().link_name = elem.first;
    thrusts_msg->thrusts.back().thrust = std::max(thrusts(idx), 0.0);
  }

  tar_thrusts_pub_->publish(std::move(thrusts_msg));
}

void ControllerNode::publishDeflections(const builtin_interfaces::msg::Time& stamp, const Eigen::VectorXd& deflections)
{
  assert(static_cast<size_t>(deflections.size()) == drone_.fixed_wing->numControlSurfaces());

  auto tar_angles_msg = std::make_unique<tobas_msgs::msg::JointCommandArray>();
  tar_angles_msg->header.stamp = stamp;

  for (const auto& [idx, cs_item] : std::views::enumerate(drone_.fixed_wing->control_surfaces)) {
    const auto& link_name = cs_item.first;
    const auto& joint = tree_.getSegment(link_name)->second.segment.joint();

    tar_angles_msg->commands.emplace_back();
    tar_angles_msg->commands.back().name = joint.name;
    tar_angles_msg->commands.back().data = deflections(idx);
  }

  tar_angles_pub_->publish(std::move(tar_angles_msg));
}

bool ControllerNode::isCommandAccepted(const tobas_command_msgs::msg::Priority& priority)
{
  if (!topics_received_) {
    TOBAS_WARN_THROTTLE(kIgnoreCmdMsgPeriod, "The command is ignored because some topics have not been received yet.");
    return false;
  }

  if (!arming_->data) {
    TOBAS_WARN_THROTTLE(kIgnoreCmdMsgPeriod, "The command is ignored because the vehicle is disarmed.");
    return false;
  }

  if (!cmd_priority_handler_.update(priority.data, now())) {
    TOBAS_WARN_THROTTLE(kIgnoreCmdMsgPeriod, "The command is ignored because of the its priority.");
    return false;
  }

  return true;
}

void ControllerNode::droneCb(const Drone::ConstSharedPtr& drone)
{
  drone_ = *drone;

  if (tree_received_) {
    if (!initialize()) {
      TOBAS_FATAL("Error occurred while initializing controller.");
      return;
    }
  }

  drone_received_ = true;
}

void ControllerNode::treeCb(const kdl::Tree::ConstSharedPtr& tree)
{
  tree_ = *tree;

  if (drone_received_) {
    if (!initialize()) {
      TOBAS_FATAL("Error occurred while initializing controller.");
      return;
    }
  }

  tree_received_ = true;
}

void ControllerNode::armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming)
{
  arming_ = arming;

  if (!arming->data) {
    thrusts_.reset();
    deflections_.reset();
  }
}

void ControllerNode::airPressureCb(const tobas_msgs::msg::FluidPressure::ConstSharedPtr& pressure)
{
  air_pressure_ = pressure;
}

void ControllerNode::odomCb(const tobas_msgs::OdometryWithCovarianceStamped::ConstSharedPtr& odom_flu)
{
  if (!odom_flu_) {
    odom_flu_ = odom_flu;
    return;
  }

  // Compute elapsed time and update odometry.
  const auto dt = (odom_flu->header.stamp - odom_flu_->header.stamp).seconds();
  odom_flu_ = odom_flu;

  if (deflections_) {
    publishThrusts(odom_flu->header.stamp, *thrusts_);
    publishDeflections(odom_flu->header.stamp, *deflections_);
  }
}

void ControllerNode::manualCmdCb(const tobas_command_msgs::msg::ElevAileRudThrottle::ConstSharedPtr& manual_cmd)
{
  if (!isCommandAccepted(manual_cmd->priority)) {
    return;
  }

  // TODO: Stop the outer control loop.

  // Set command
  *thrusts_ = Eigen::VectorXd::Zero(drone_.prop->numRotors());
  int i = 0;
  for (const auto& [link_name, _] : drone_.prop->rotors) {
    const auto thrust_at_full_throt = drone_.prop->thrustFromThrottle(link_name, kMaxThrot);
    thrusts_->operator[](i) = thrust_at_full_throt * manual_cmd->throttle;
    i++;
  }
  *deflections_ = Eigen::VectorXd::Zero(drone_.fixed_wing->numControlSurfaces());
  i = 0;
  for (const auto& [link_name, _] : drone_.fixed_wing->control_surfaces) {
    deflections_->operator[](i) = 0.0; // TODO: set values
    i++;
  }
}

void ControllerNode::checkTopicsTimerCb()
{
  if (!drone_received_) {
    TOBAS_WARN("Waiting for \"", topic::kDrone, "\".");
    return;
  }

  if (!tree_received_) {
    TOBAS_WARN("Waiting for \"", topic::kKdlTree, "\".");
    return;
  }

  if (!air_pressure_) {
    TOBAS_WARN("Waiting for \"", topic::kAirPressure, "\".");
    return;
  }

  if (!odom_flu_) {
    TOBAS_WARN("Waiting for \"", topic::kOdometry, "\".");
    return;
  }

  topics_received_ = true;
  check_topics_timer_->cancel();
}
}  // namespace fixed_wing
}  // namespace tobas

RCLCPP_COMPONENTS_REGISTER_NODE(tobas::fixed_wing::ControllerNode)
