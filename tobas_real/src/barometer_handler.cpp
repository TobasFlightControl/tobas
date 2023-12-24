#include <boost/property_tree/ini_parser.hpp>
#include <sensor_msgs/FluidPressure.h>

#include <tobas_std_tools/math.hpp>
#include <tobas_ros_tools/rosparam.hpp>
#include <tobas_ros_tools/console_message.hpp>
#include <tobas_ros_tools/exception.hpp>

#include "../include/tobas_real/barometer_handler.hpp"
#include "../include/tobas_real/common.hpp"

using namespace std;

namespace tobas_real
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
  {
    ROS_THROW_NAMED(name_, "Barometer test failed.");
  }

  registerPublishers();
  registerSubscribers();

  main_timer_ = nh_.createTimer(update_rate_, &self::mainTimerCb, this);
}

void BarometerHandler::getRosParams()
{
  tobas_ros::getParam(pnh_, "update_rate", update_rate_, kDefaultUpdateRate);
}

void BarometerHandler::registerPublishers()
{
  bar_pub_ = nh_.advertise<sensor_msgs::FluidPressure>(tobas::kAirPressureTopic, 1);
}

void BarometerHandler::registerSubscribers()
{
  super::registerSubscribers();
}

void BarometerHandler::readConfig()
{
  boost::property_tree::ptree pt;
  boost::property_tree::ini_parser::read_ini(kConfigPath, pt);

  pressure_noise_density_ = pt.get<double>(kConfigKey_PressureNoiseDensity);
}

void BarometerHandler::eventCb(const tobas_msgs::EventConstPtr& event)
{
  switch (event->data)
  {
    case tobas_msgs::Event::STOP:
      nh_.shutdown();
      main_timer_.stop();
      break;
    default:
      break;
  }
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
  bar_msg->header.frame_id = "barometer_frame";
  bar_msg->fluid_pressure = pressure;
  bar_msg->variance = tobas_std::sqr(pressure_noise_density_) * update_rate_;  // [Pa^2]

  // メッセージを発行
  bar_pub_.publish(bar_msg);
}
}  // namespace tobas_real
