#include <tobas_hal_core/constants.hpp>
#include <tobas_hal_msgs/FluidPressure.h>

#include "../include/tobas_a1_ros/baro_driver.hpp"

using namespace std;

namespace a1
{
BaroDriver::BaroDriver(ros::NodeHandle& nh, ros::NodeHandle& pnh, const string& name) : super(nh, pnh, name)
{
  if (!baro_.initialize())
    TOBAS_EXIT("Failed to initialize Barometer.");

  baro_pub_ = nh_.advertise<tobas_hal_msgs::FluidPressure>(hal::kAirPressureTopic, 1);
  main_timer_ = nh_.createTimer(kSamplingRate, &self::mainTimerCb, this);
}

void BaroDriver::mainTimerCb(const ros::TimerEvent& event)
{
  // Create messages
  const auto msg = boost::make_shared<tobas_hal_msgs::FluidPressure>();

  // Fill headers
  msg->header.stamp = event.current_real;

  // Read sensor
  if (!baro_.readPressure(msg->fluid_pressure))
  {
    TOBAS_FATAL("Failed to read barometer.");
    return;
  }

  // Publish message
  baro_pub_.publish(msg);
}
}  // namespace a1
