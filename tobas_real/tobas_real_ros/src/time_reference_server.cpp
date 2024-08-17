#include <sensor_msgs/TimeReference.h>

#include <tobas_constants/constants.hpp>

#include "../include/tobas_real_ros/time_reference_server.hpp"

using namespace std;

namespace tobas_real_ros
{
TimeReferenceServer::TimeReferenceServer(const rclcpp::NodeOptions& options) : super(name, options)
{
  time_ref_pub_ = createPublisher<sensor_msgs::msg::TimeReference>(tobas::kTimeReferenceTopic);
  main_timer_ = createTimer(kUpdateRate, &self::mainTimerCb, this);
}

void TimeReferenceServer::mainTimerCb()
{
  const auto time_ref = std::make_unique<sensor_msgs::msg::TimeReference>();
  const auto now = get_clock()->now();
  time_ref->header.stamp = now;
  time_ref->time_ref = now;
  time_ref_pub_->publish(time_ref);
}
}  // namespace tobas_real_ros
