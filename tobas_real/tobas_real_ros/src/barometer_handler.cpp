#include <sensor_msgs/msg/fluid_pressure.hpp>

#include <tobas_constants/constants.hpp>
#include <tobas_hal_core/constants.hpp>

#include "../include/tobas_real_ros/barometer_handler.hpp"
#include "../include/tobas_real_ros/common.hpp"

using namespace std;

namespace tobas_real_ros
{
BarometerHandler::BarometerHandler(const rclcpp::NodeOptions& options) : super(name, options)
{
  bar_pub_ = createPublisher<sensor_msgs::msg::FluidPressure>(tobas::kAirPressureTopic);
  bar_sub_ = createSubscriber(hal::kAirPressureTopic, &self::airPressureCb, this);
}

void BarometerHandler::airPressureCb(const tobas_hal_msgs::msg::FluidPressure::ConstSharedPtr& bar_raw)
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
  const auto bar_msg =std::make_unique<sensor_msgs::msg::FluidPressure>();
  bar_msg->header = bar_raw->header;
  bar_msg->fluid_pressure = bar_raw->fluid_pressure;
  bar_msg->variance = pressure_noise_.noiseVariance();

  // Publish message
  bar_pub_->publish(bar_msg);
}
}  // namespace tobas_real_ros
