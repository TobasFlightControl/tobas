// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <tobas_constants/ros_interface.hpp>
#include <tobas_node/node.hpp>

#include <tobas_msgs_adapter/gnss.hpp>

using namespace std::chrono_literals;

namespace tobas
{
class FakeGnssPublisherNode : public BaseNode
{
  static constexpr auto kSamplingPeriod = 200ms;

  static constexpr double kDefaultPosStddev = 3.0;  // [m]
  static constexpr double kDefaultVelStddev = 0.3;  // [m/s]

  using self = FakeGnssPublisherNode;
  using super = BaseNode;

public:
  explicit FakeGnssPublisherNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  double pos_stddev_;
  double vel_stddev_;

  ros2::PublisherPtr<tobas_msgs::Gnss> gnss_pub_;
  ros2::TimerPtr timer_;

  void timerCb();
};

FakeGnssPublisherNode::FakeGnssPublisherNode(const rclcpp::NodeOptions& options)
  : super("fake_batt_publisher", nodeOptions_Default(options))
{
  pos_stddev_ = getDoubleParam("position_stddev", kDefaultPosStddev);
  vel_stddev_ = getDoubleParam("velocity_stddev", kDefaultVelStddev);

  gnss_pub_ = createPublisher<tobas_msgs::Gnss>(topic::kGnss);
  timer_ = createTimer(kSamplingPeriod, &self::timerCb, this);
}

void FakeGnssPublisherNode::timerCb()
{
  auto gnss_msg = std::make_unique<tobas_msgs::Gnss>();
  gnss_msg->header.stamp = now();
  gnss_msg->fix_type = tobas_msgs::msg::Gnss::FIX_3D;
  gnss_msg->latitude = 0.0;
  gnss_msg->longitude = 0.0;
  gnss_msg->altitude = 0.0;
  gnss_msg->ground_speed.setZero();
  gnss_msg->position_covariance = Eigen::Vector3d::Constant(pos_stddev_).asDiagonal();
  gnss_msg->velocity_covariance = Eigen::Vector3d::Constant(vel_stddev_).asDiagonal();

  gnss_pub_->publish(std::move(gnss_msg));
}
}  // namespace tobas

RCLCPP_COMPONENTS_REGISTER_NODE(tobas::FakeGnssPublisherNode)
