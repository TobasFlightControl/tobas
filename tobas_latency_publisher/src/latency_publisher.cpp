#include <tobas_constants/constants.hpp>
#include <tobas_msgs/Latency.h>

#include "../include/tobas_latency_publisher/latency_publisher.hpp"

using namespace std;

namespace tobas_latency_publisher
{
LatencyPublisher::LatencyPublisher(const rclcpp::NodeOptions& options) : super(node, pnh, name)
{
  latency_pub_ = createPublisher<tobas_msgs::Latency>(tobas::kLatencyTopic);
  throttles_sub_ = createSubscriber(tobas::kThrottlesCmdTopic, &self::throttlesCb, this);
}

void LatencyPublisher::throttlesCb(const tobas_msgs::ThrottleArray::ConstSharedPtr& msg)
{
  const auto latency =std::make_unique<tobas_msgs::Latency>();
  const auto cur_time = node->get_clock()->now();
  latency->header.stamp = cur_time;
  latency->data = cur_time - msg->header.stamp;
  latency_pub_->publish(latency);
}
}  // namespace tobas_latency_publisher
