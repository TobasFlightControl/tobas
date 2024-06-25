#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.hpp>

#include "../include/tobas_topic_throttle/battery_throttle.hpp"

namespace tobas_topic_throttle
{
class BatteryThrottleNodelet : public nodelet::Nodelet
{
public:
  void onInit() override
  {
    node_.reset(new BatteryThrottle(getNodeHandle()));
  }

private:
  std::shared_ptr<BatteryThrottle> node_;
};
}  // namespace tobas_topic_throttle

PLUGINLIB_EXPORT_CLASS(tobas_topic_throttle::BatteryThrottleNodelet, nodelet::Nodelet);
