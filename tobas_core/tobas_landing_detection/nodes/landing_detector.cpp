// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <tobas_constants/ros_interface.hpp>
#include <tobas_dsp/low_pass_filter.hpp>
#include <tobas_kdl/tree_mass_holder.hpp>
#include <tobas_node/node.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_std_tools/universal_constants.hpp>

#include <tobas_kdl_msgs_adapter/tree.hpp>
#include <tobas_kdl_msgs_adapter/wrench_stamped.hpp>
#include <tobas_msgs/msg/landed_state.hpp>

using namespace std::chrono_literals;

namespace tobas
{
class LandingDetectorNode : public BaseNode
{
  using self = LandingDetectorNode;
  using super = BaseNode;

  static constexpr auto kPublishPeriod = 1s;
  static constexpr double kDistForceLpfCutoff = 1.;        // [Hz]
  static constexpr double kTakeoffWeightRateThresh = 0.3;  // [-]
  static constexpr auto kTakeoffDetectTimeThresh = 200ms;
  static constexpr double kLandWeightRateThresh = 0.7;  // [-]
  static constexpr auto kLandDetectTimeThresh = 1s;

public:
  explicit LandingDetectorNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  bool landed_ = true;
  tobas_kdl_msgs::WrenchStamped::ConstSharedPtr dist_force_;
  rclcpp::Time t_last_no_change_;  // Last time the upward vertical force crossed the threshold
  dsp::LowPassFilter<double> force_z_lpf_;

  kdl::Tree tree_;
  kdl::TreeMassHolder mass_holder_;

  ros2::PublisherPtr<tobas_msgs::msg::LandedState> landed_pub_;

  ros2::SubscriberPtr<kdl::Tree> tree_sub_;
  ros2::SubscriberPtr<tobas_kdl_msgs::WrenchStamped> dist_force_sub_;

  ros2::TimerPtr publish_timer_;

  void publishCurrentLandedState(const builtin_interfaces::msg::Time& stamp);
  void changeState(bool landed, const builtin_interfaces::msg::Time& stamp);

  void treeCb(const kdl::Tree::ConstSharedPtr& tree);
  void disturbanceForceCb(const tobas_kdl_msgs::WrenchStamped::ConstSharedPtr& dist_force);

  void publishTimerCb();
};

LandingDetectorNode::LandingDetectorNode(const rclcpp::NodeOptions& options)
  : super("landing_detector", nodeOptions_Default(options)), mass_holder_(tree_)
{
  static_assert(kTakeoffWeightRateThresh < kLandWeightRateThresh);  // Hysteresis is required.

  force_z_lpf_.setCutoffFrequency(kDistForceLpfCutoff);

  landed_pub_ = createPublisher<tobas_msgs::msg::LandedState>(topic::kLanded);

  tree_sub_ = createSubscriber(topic::kKdlTree, &self::treeCb, this, true, true);
  dist_force_sub_ = createSubscriber(topic::kDisturbanceForce, &self::disturbanceForceCb, this);
}

void LandingDetectorNode::publishCurrentLandedState(const builtin_interfaces::msg::Time& stamp)
{
  auto msg = std::make_unique<tobas_msgs::msg::LandedState>();
  msg->header.stamp = stamp;
  msg->landed = landed_;
  landed_pub_->publish(std::move(msg));
}

void LandingDetectorNode::changeState(bool landed, const builtin_interfaces::msg::Time& stamp)
{
  landed_ = landed;
  t_last_no_change_ = stamp;
  publishCurrentLandedState(stamp);
  publish_timer_->reset();
}

void LandingDetectorNode::treeCb(const kdl::Tree::ConstSharedPtr& tree)
{
  tree_ = *tree;
  mass_holder_.updateInternalDataStructures();
}

void LandingDetectorNode::disturbanceForceCb(const tobas_kdl_msgs::WrenchStamped::ConstSharedPtr& dist_force)
{
  // Verify that the KDL tree has been received
  if (tree_.getNrOfSegments() == 0) {
    return;
  }

  // Get the latest vertical force wrt. the world frame
  const auto& force_z = dist_force->wrench.force.z();

  // First message
  if (!dist_force_) {
    dist_force_ = dist_force;
    t_last_no_change_ = dist_force->header.stamp;
    force_z_lpf_.setValue(force_z);
    publish_timer_ = createTimer(kPublishPeriod, &self::publishTimerCb, this);
    return;
  }

  // Smooth the vertical force
  const auto dt = (dist_force->header.stamp - dist_force_->header.stamp).seconds();
  force_z_lpf_.update(force_z, dt);

  // Update the latest message
  dist_force_ = dist_force;

  // Change the takeoff/landing state if the upward external force (ground reaction force)
  // stays beyond the threshold ratio to weight for a fixed time.
  const auto& cur_time = dist_force->header.stamp;
  const auto& force_z_filt = force_z_lpf_.getValue();
  const auto weight = mass_holder_.getMass() * st::kGravity;
  if (landed_) {
    const auto force_z_thresh = weight * kTakeoffWeightRateThresh;
    if (force_z_filt < force_z_thresh) {
      if (cur_time - t_last_no_change_ > kTakeoffDetectTimeThresh) {
        TOBAS_INFO("Takeoff detected.");
        changeState(false, cur_time);
      }
    }
    else {
      t_last_no_change_ = cur_time;
    }
  }
  else {
    const auto force_z_thresh = weight * kLandWeightRateThresh;
    if (force_z_filt > force_z_thresh) {
      if (cur_time - t_last_no_change_ > kLandDetectTimeThresh) {
        TOBAS_INFO("Landing detected.");
        changeState(true, cur_time);
      }
    }
    else {
      t_last_no_change_ = cur_time;
    }
  }
}

void LandingDetectorNode::publishTimerCb()
{
  publishCurrentLandedState(now());
}
}  // namespace tobas

RCLCPP_COMPONENTS_REGISTER_NODE(tobas::LandingDetectorNode)
