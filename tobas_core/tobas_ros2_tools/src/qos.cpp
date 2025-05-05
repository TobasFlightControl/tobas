#include "tobas_ros2_tools/qos.hpp"

namespace ros2
{
rclcpp::QoS makeQoS(bool latch, bool reliable, size_t queue_size)
{
  auto qos = rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, queue_size));

  if (latch) {
    qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
  }
  else {
    qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
  }

  if (reliable) {
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  }
  else {
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  }

  return qos;
}
}  // namespace ros2
