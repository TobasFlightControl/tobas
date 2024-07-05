#include <sensor_msgs/FluidPressure.h>

#include <tobas_math/core.hpp>
#include <tobas_std_tools/time.hpp>
#include <tobas_ros_tools/rosparam.hpp>
#include <tobas_tools/constants.hpp>

#include "../include/tobas_navio_ros/barometer_handler.hpp"
#include "../include/tobas_navio_ros/common.hpp"

using namespace std;

namespace tobas_navio_ros
{
BarometerHandler::BarometerHandler(ros::NodeHandle& nh, ros::NodeHandle& pnh, const string& name) : super(nh, pnh, name)
{
  PRINT_DEBUG("BarometerHandler::BarometerHandler");

  initialize();

  bar_pub_ = nh_.advertise<sensor_msgs::FluidPressure>(tobas::kAirPressureTopic, 1);
  main_timer_ = nh_.createTimer(kSamplingRate, &self::mainTimerCb, this);

  PRINT_DEBUG("/BarometerHandler::BarometerHandler");
}

void BarometerHandler::initialize()
{
  if (!barometer_.initialize())
    TOBAS_EXIT("Failed to initialize barometer.");

  if (!barometer_.update())
    TOBAS_EXIT("Failed to update barometer.");

  const auto pressure = barometer_.getPressure();
  pressure_noise_.initialize(kWindowSize, kHpfCutoff, pressure);
}

void BarometerHandler::mainTimerCb(const ros::TimerEvent& event)
{
  // バロメータを更新
  if (!barometer_.update())
  {
    TOBAS_ERROR_THROTTLE(kErrorPeriod, "Failed to update barometer.");
    return;
  }

  // 気圧を求める
  const auto pressure = barometer_.getPressure();
  if (pressure < kMinAirPressure || kMaxAirPressure < pressure)
  {
    TOBAS_ERROR_THROTTLE(kErrorPeriod, "Strange air pressure: ", pressure, " [Pa]");
    return;
  }

  // Update noise filter
  const auto dt = (event.current_real - event.last_real).toSec();
  pressure_noise_.update(pressure, dt);

  // メッセージを作成
  const auto bar_msg = boost::make_shared<sensor_msgs::FluidPressure>();
  bar_msg->header.stamp = event.current_real;
  bar_msg->fluid_pressure = pressure;
  bar_msg->variance = pressure_noise_.noiseVariance();

  // メッセージを発行
  bar_pub_.publish(bar_msg);
}
}  // namespace tobas_navio_ros
