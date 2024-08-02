#include <sensor_msgs/TimeReference.h>

#include <tobas_tools/constants.hpp>

#include "../include/tobas_real_ros/time_reference_server.hpp"

using namespace std;

namespace tobas_real_ros
{
TimeReferenceServer::TimeReferenceServer(, const string& name)
  : super(node, pnh, name)
{
  time_ref_pub_ = node_.advertise<sensor_msgs::msg::TimeReference>(tobas::kTimeReferenceTopic, 1);
  main_timer_ = node_.createTimer(kUpdateRate, &self::mainTimerCb, this);
}

void TimeReferenceServer::mainTimerCb(const rclcpp::TimerEvent& event)
{
  const auto time_ref = make_unique<sensor_msgs::msg::TimeReference>();
  time_ref->header.stamp = event.current_real;
  time_ref->time_ref = event.current_real;
  time_ref_pub_.publish(time_ref);
}
}  // namespace tobas_real_ros
