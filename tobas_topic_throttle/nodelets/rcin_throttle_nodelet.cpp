#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.hpp>

#include "../include/tobas_topic_throttle/rcin_throttle.hpp"

namespace tobas_topic_throttle
{
class RCInputThrottleNodelet : public nodelet::Nodelet
{
public:
  void onInit() override
  {
    node_.reset(new RCInputThrottle(getNodeHandle()));
  }

private:
  std::shared_ptr<RCInputThrottle> node_;
};
}  // namespace tobas_topic_throttle

PLUGINLIB_EXPORT_CLASS(tobas_topic_throttle::RCInputThrottleNodelet, nodelet::Nodelet);
