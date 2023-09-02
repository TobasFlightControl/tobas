#include <boost/property_tree/ini_parser.hpp>
#include <sensor_msgs/FluidPressure.h>

#include <dh_std_tools/math.hpp>
#include <dh_ros_tools/rosparam.hpp>
#include <dh_ros_tools/rate.hpp>

#include "../include/tobas_real/barometer_handler.hpp"
#include "../include/tobas_real/common.hpp"

namespace tobas_real
{
BarometerHandler::BarometerHandler(ros::NodeHandle nh, ros::NodeHandle pnh) : super(nh, pnh)
{
  getRosParams();

  barometer_.initialize();
  if (!barometer_.testConnection())
  {
    rosthrow("Barometer test failed.");
  }

  registerPublishers();
  registerSubscribers();

  main_timer_ =
    nh_.createTimer(ros::Duration(1 / kUpdateRate), &BarometerHandler::mainTimerCb, this);
}

void BarometerHandler::getRosParams()
{
}

void BarometerHandler::registerPublishers()
{
  bar_pub_ = nh_.advertise<sensor_msgs::FluidPressure>("air_pressure", 1);
}

void BarometerHandler::registerSubscribers()
{
  event_sub_ = nh_.subscribe("event", 1, &BarometerHandler::eventCb, this);
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
    case tobas_msgs::Event::SHUTDOWN:
      // nh_.shutdown();
      break;
    default:
      break;
  }
}

void BarometerHandler::mainTimerCb(const ros::TimerEvent& event)
{
  // バロメータを更新
  barometer_.refreshPressure();
  usleep(kWaitToUpdateSensor);  // この待ち時間が必須
  barometer_.readPressure();
  barometer_.calculatePressureAndTemperature();

  // 気圧を求める
  const auto pressure = barometer_.getPressure() * 100;  // mbar -> Pa
  if (pressure < kMinAirPressure || kMaxAirPressure < pressure)
  {
    rosError("Strange air pressure: " << pressure << " [Pa]");
    return;
  }

  // メッセージを作成
  const auto bar_msg = boost::make_shared<sensor_msgs::FluidPressure>();
  bar_msg->header.stamp = event.current_real;
  bar_msg->header.frame_id = "barometer_frame";
  bar_msg->fluid_pressure = pressure;
  bar_msg->variance = dh_std::sqr(pressure_noise_density_) * kUpdateRate;  // [Pa^2]

  // メッセージを発行
  bar_pub_.publish(bar_msg);
}
}  // namespace tobas_real
