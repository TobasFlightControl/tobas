#include <sensor_msgs/TimeReference.h>

#include <tobas_tools/constants.hpp>

#include "../include/tobas_real_ros/time_reference_server.hpp"

using namespace std;

namespace tobas_real_ros
{
TimeReferenceServer::TimeReferenceServer(
  const ros::NodeHandle& nh,
  const ros::NodeHandle& pnh,
  const string& name)
  : super(nh, pnh, name)
{
  time_ref_pub_ = nh_.advertise<sensor_msgs::TimeReference>(tobas::kTimeReferenceTopic, 1);
  main_timer_ = nh_.createTimer(kUpdateRate, &self::mainTimerCb, this);
}

void TimeReferenceServer::mainTimerCb(const ros::TimerEvent& event)
{
  const auto time_ref = boost::make_shared<sensor_msgs::TimeReference>();
  time_ref->header.stamp = event.current_real;
  time_ref->time_ref = event.current_real;
  time_ref_pub_.publish(time_ref);
}
}  // namespace tobas_real_ros
