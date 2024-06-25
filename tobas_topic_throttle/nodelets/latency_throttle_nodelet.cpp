#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.hpp>

#include "../include/tobas_topic_throttle/latency_throttle.hpp"

namespace tobas_topic_throttle
{
class LatencyThrottleNodelet : public nodelet::Nodelet
{
public:
  void onInit() override
  {
    node_.reset(new LatencyThrottle(getNodeHandle()));
  }

private:
  std::shared_ptr<LatencyThrottle> node_;
};
}  // namespace tobas_topic_throttle

PLUGINLIB_EXPORT_CLASS(tobas_topic_throttle::LatencyThrottleNodelet, nodelet::Nodelet);
