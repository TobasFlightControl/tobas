#include <sensor_msgs/FluidPressure.h>

#include <tobas_std_tools/math.hpp>
#include <tobas_ros_tools/rosparam.hpp>
#include <tobas_ros_tools/console_message.hpp>
#include <tobas_std_tools/property_tree.hpp>
#include <tobas_ros_tools/exception.hpp>
#include <tobas_tools/constants.hpp>

#include "../include/tobas_navio_ros/barometer_handler.hpp"
#include "../include/tobas_navio_ros/common.hpp"

using namespace std;

namespace tobas_navio_ros
{
BarometerHandler::BarometerHandler(
  const ros::NodeHandle& nh,
  const ros::NodeHandle& pnh,
  const string& name)
  : super(nh, pnh, name)
{
  getRosParams();

  barometer_.initialize();
  if (!barometer_.testConnection())
    ROS_EXIT_NAMED(nh_, name_, "Barometer test failed.");

  registerPublishers();
  registerSubscribers();

  main_timer_ = nh_.createTimer(kSamplingRate, &self::mainTimerCb, this);
}

void BarometerHandler::getRosParams()
{
}

void BarometerHandler::registerPublishers()
{
  bar_pub_ = nh_.advertise<sensor_msgs::FluidPressure>(tobas::kAirPressureTopic, 1);
}

void BarometerHandler::registerSubscribers()
{
}

void BarometerHandler::readConfig()
{
  tobas_std::PropertyTree pt(kConfigPath);
  pt.get(kConfigKey_PressureNoiseDensity, pressure_noise_density_);
}

void BarometerHandler::mainTimerCb(const ros::TimerEvent& event)
{
  // バロメータを更新
  barometer_.refreshPressure();
  usleep(kWaitToRefreshBarometer);  // この待ち時間が必須
  barometer_.readPressure();
  barometer_.calculatePressureAndTemperature();

  // 気圧を求める
  const auto pressure = barometer_.getPressure() * 100;  // mbar -> Pa
  if (pressure < kMinAirPressure || kMaxAirPressure < pressure)
  {
    rosError(name_, "Strange air pressure: " << pressure << " [Pa]");
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
