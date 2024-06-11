#include <sensor_msgs/FluidPressure.h>

#include <tobas_std_tools/math.hpp>
#include <tobas_std_tools/time.hpp>
#include <tobas_ros_tools/rosparam.hpp>
#include <tobas_std_tools/property_tree.hpp>
#include <tobas_tools/constants.hpp>

#include "../include/tobas_navio_ros/barometer_handler.hpp"
#include "../include/tobas_navio_ros/common.hpp"

using namespace std;

namespace tobas_navio_ros
{
BarometerHandler::BarometerHandler(const ros::NodeHandle& nh, const ros::NodeHandle& pnh, const string& name)
  : super(nh, pnh, name)
{
  PRINT_DEBUG("BarometerHandler::BarometerHandler");

  reloadConfig();

  barometer_.initialize();
  if (!barometer_.testConnection())
    TOBAS_EXIT("Barometer test failed.");

  bar_pub_ = nh_.advertise<sensor_msgs::FluidPressure>(tobas::kAirPressureTopic, 1);

  reload_config_srv_ = nh_.advertiseService(name + tobas::kReloadConfigSrvSuffix, &self::reloadConfigCb, this);
  main_timer_ = nh_.createTimer(kSamplingRate, &self::mainTimerCb, this);

  PRINT_DEBUG("/BarometerHandler::BarometerHandler");
}

bool BarometerHandler::reloadConfig()
{
  tobas_std::PropertyTree pt(kConfigPath);

  if (!pt.get(kConfigKey_PressureNoiseDensity, pressure_noise_density_, kDefaultPressureNoiseDensity))
  {
    TOBAS_ERROR("Failed to get ", kConfigKey_PressureNoiseDensity, ".");
    return false;
  }

  return true;
}

bool BarometerHandler::reloadConfigCb(std_srvs::TriggerRequest&, std_srvs::TriggerResponse& res)
{
  if (!reloadConfig())
  {
    res.success = false;
    res.message = "Failed to reload configurations.";
    return true;
  }

  res.success = true;
  return true;
}

void BarometerHandler::mainTimerCb(const ros::TimerEvent& event)
{
  // バロメータを更新
  barometer_.refreshPressure();
  tobas_std::msleep(kWaitToRefreshBarometer);  // この待ち時間が必須
  barometer_.readPressure();
  barometer_.calculatePressureAndTemperature();

  // 気圧を求める
  const auto pressure = barometer_.getPressure();
  if (pressure < kMinAirPressure || kMaxAirPressure < pressure)
  {
    TOBAS_ERROR("Strange air pressure: ", pressure, " [Pa]");
    return;
  }

  // メッセージを作成
  const auto bar_msg = boost::make_shared<sensor_msgs::FluidPressure>();
  bar_msg->header.stamp = event.current_real;
  bar_msg->fluid_pressure = pressure;
  bar_msg->variance = tobas_std::sqr(pressure_noise_density_) * kSamplingRate;  // [Pa^2]

  // メッセージを発行
  bar_pub_.publish(bar_msg);
}
}  // namespace tobas_navio_ros
