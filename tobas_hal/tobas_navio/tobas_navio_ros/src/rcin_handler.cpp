#include <tobas_hal_core/constants.hpp>
#include <tobas_hal_msgs/Sbus.h>

#include "../include/tobas_navio_ros/rcin_handler.hpp"
#include "../include/tobas_navio_ros/common.hpp"

using namespace std;

namespace tobas_navio_ros
{
RCInputHandler::RCInputHandler(ros::NodeHandle& nh, ros::NodeHandle& pnh, const string& name) : super(nh, pnh, name)
{
  if (!rcin_.initialize())
    TOBAS_EXIT("Failed to initialize RC input driver.");

  rcin_pub_ = nh_.advertise<tobas_hal_msgs::Sbus>(hal::kSbusTopic, 1);
  main_timer_ = nh_.createTimer(kSamplingRate, &self::mainTimerCb, this);
}

void RCInputHandler::mainTimerCb(const ros::TimerEvent& event)
{
  // Create message
  const auto rcin_msg = boost::make_shared<tobas_hal_msgs::Sbus>();
  rcin_msg->header.stamp = event.current_real;

  // Read each channel
  for (size_t ch = 0; ch < navio::RCInput::channelCount(); ++ch)
  {
    if (!rcin_.read(ch))
    {
      TOBAS_ERROR_THROTTLE(kErrorPeriod, "Failed to read RC input.");
      return;
    }
    rcin_msg->data[ch] = rcin_.getPeriod();
  }

  // Publish message
  rcin_pub_.publish(rcin_msg);
}
}  // namespace tobas_navio_ros
