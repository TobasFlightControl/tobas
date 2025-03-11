#include <tobas_constants/constants.hpp>

#include "../include/tobas_tools/imu_sampling_time_publisher.hpp"

namespace tobas
{
ImuSamplingTimePublisher::ImuSamplingTimePublisher()
{
}

void ImuSamplingTimePublisher::initialize(rclcpp::Node::SharedPtr node, const rclcpp::Time& cur_time)
{
  last_time_ = cur_time;

  pub_ = ros2::createPublisher<tobas_msgs::msg::Latency>(node, tobas::kImuSamplingTimeTopic);
}

void ImuSamplingTimePublisher::publish(const rclcpp::Time& cur_time)
{
  auto msg = std::make_unique<tobas_msgs::msg::Latency>();
  msg->header.stamp = cur_time;
  msg->data = cur_time - last_time_;
  pub_->publish(std::move(msg));

  last_time_ = cur_time;
}
}  // namespace tobas
