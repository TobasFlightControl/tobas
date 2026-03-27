#pragma once

#include <rclcpp/rclcpp.hpp>

namespace tobas
{
namespace ros2
{
namespace qos
{
static constexpr bool kDefaultLatch = false;
static constexpr bool kDefaultReliable = false;
static constexpr size_t kDefaultQueueSize = 1;

class QoS : public rclcpp::QoS
{
public:
  explicit QoS(bool latch, bool reliable, size_t queue_size);
};

/* The default topic QoS. */
class DefaultQoS : public QoS
{
public:
  explicit DefaultQoS();
};
}  // namespace qos
}  // namespace ros2
}  // namespace tobas
