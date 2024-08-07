#include <sensor_msgs/FluidPressure.h>

#include <tobas_constants/constants.hpp>
#include <tobas_hal_core/constants.hpp>

#include "../include/tobas_real_ros/barometer_handler.hpp"
#include "../include/tobas_real_ros/common.hpp"

using namespace std;

namespace tobas_real_ros
{
BarometerHandler::BarometerHandler(, const string& name) : super(node, pnh, name)
{
  bar_pub_ = node_.advertise<sensor_msgs::msg::FluidPressure>(tobas::kAirPressureTopic, 1);
  bar_sub_ = node_.subscribe(hal::kAirPressureTopic, 1, &self::airPressureCb, this, tcpNoDelay());
}

void BarometerHandler::airPressureCb(const tobas_hal_msgs::FluidPressureConstPtr& bar_raw)
{
  // Initialize
  if (bar_raw_ == nullptr)
  {
    pressure_noise_.initialize(kWindowSize, kHpfCutoff, bar_raw->fluid_pressure);
    bar_raw_ = bar_raw;
    return;
  }

  // Compute time difference
  const auto dt = (bar_raw->header.stamp - bar_raw_->header.stamp).seconds();
  bar_raw_ = bar_raw;

  // Validate
  if (bar_raw->fluid_pressure < kMinAirPressure || kMaxAirPressure < bar_raw->fluid_pressure)
  {
    TOBAS_ERROR_THROTTLE(kErrorPeriod, "Strange air pressure: ", bar_raw->fluid_pressure, " [Pa]");
    return;
  }

  // Update noise filter
  pressure_noise_.update(bar_raw->fluid_pressure, dt);

  // Create message
  const auto bar_msg = make_unique<sensor_msgs::msg::FluidPressure>();
  bar_msg->header = bar_raw->header;
  bar_msg->fluid_pressure = bar_raw->fluid_pressure;
  bar_msg->variance = pressure_noise_.noiseVariance();

  // Publish message
  bar_pub_.publish(bar_msg);
}
}  // namespace tobas_real_ros
