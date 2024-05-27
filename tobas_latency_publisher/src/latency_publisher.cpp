#include <tobas_tools/constants.hpp>
#include <tobas_msgs/Latency.h>

#include "../include/tobas_latency_publisher/latency_publisher.hpp"

using namespace std;

namespace tobas_latency_publisher
{
LatencyPublisher::LatencyPublisher(const ros::NodeHandle& nh, const ros::NodeHandle& pnh, const string& name)
  : super(nh, pnh, name)
{
  latency_pub_ = nh_.advertise<tobas_msgs::Latency>(tobas::kLatencyTopic, 1);
  tar_rot_speeds_sub_ = nh_.subscribe(tobas::kRotorSpeedsCmdTopic, 1, &self::targetRotorSpeedsCb, this, tcpNoDelay());
}

void LatencyPublisher::targetRotorSpeedsCb(const tobas_msgs::RotorSpeedsConstPtr& tar_speeds)
{
  const auto latency = boost::make_shared<tobas_msgs::Latency>();
  latency->header.stamp = tar_speeds->header.stamp;
  latency->data = (ros::Time::now() - tar_speeds->header.stamp).toSec();
  latency_pub_.publish(latency);
}
}  // namespace tobas_latency_publisher
