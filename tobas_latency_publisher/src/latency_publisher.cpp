#include <tobas_tools/constants.hpp>
#include <tobas_msgs/Latency.h>

#include "../include/tobas_latency_publisher/latency_publisher.hpp"

using namespace std;

namespace tobas_latency_publisher
{
LatencyPublisher::LatencyPublisher(ros::NodeHandle& nh, ros::NodeHandle& pnh, const string& name) : super(nh, pnh, name)
{
  latency_pub_ = nh_.advertise<tobas_msgs::Latency>(tobas::kLatencyTopic, 1);
  throttles_sub_ = nh_.subscribe(tobas::kThrottlesCmdTopic, 1, &self::throttlesCb, this, tcpNoDelay());
}

void LatencyPublisher::throttlesCb(const tobas_msgs::ThrottleArrayConstPtr& msg)
{
  const auto latency = boost::make_shared<tobas_msgs::Latency>();
  const auto cur_time = ros::Time::now();
  latency->header.stamp = cur_time;
  latency->data = cur_time - msg->header.stamp;
  latency_pub_.publish(latency);
}
}  // namespace tobas_latency_publisher
