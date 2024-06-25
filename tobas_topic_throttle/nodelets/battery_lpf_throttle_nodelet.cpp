#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.hpp>

#include "../include/tobas_topic_throttle/battery_lpf_throttle.hpp"

namespace tobas_topic_throttle
{
class BatteryLPFThrottleNodelet : public nodelet::Nodelet
{
public:
  void onInit() override
  {
    node_.reset(new BatteryLPFThrottle(getNodeHandle()));
  }

private:
  std::shared_ptr<BatteryLPFThrottle> node_;
};
}  // namespace tobas_topic_throttle

PLUGINLIB_EXPORT_CLASS(tobas_topic_throttle::BatteryLPFThrottleNodelet, nodelet::Nodelet);
