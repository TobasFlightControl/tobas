#include <dh_std_tools/math.hpp>
#include <dh_ros_tools/rosparam.hpp>
#include <dh_ros_tools/rate.hpp>

#include "../../include/tobas_real/barometer_handler.hpp"

namespace tobas_real
{
constexpr double BarometerHandler::kBarNoiseStd;

BarometerHandler::BarometerHandler() : super()
{
  getRosParams();

  barometer_.initialize();
  bar_msg_.variance = dh_std::sqr(kBarNoiseStd);

  registerPublishers();
  registerSubscribers();
}

void BarometerHandler::run()
{
  dh_ros::Rate rate(kUpdateRate);

  while (ros::ok())
  {
    barometer_.refreshPressure();
    ros::Duration(kWaitTime).sleep();  // Waiting for pressure data ready
    barometer_.readPressure();
    barometer_.calculatePressureAndTemperature();

    bar_msg_.fluid_pressure = barometer_.getPressure() * 100;  // mbar -> Pa
    bar_pub_.publish(bar_msg_);

    ros::spinOnce();
    rate.sleep();
  }
}

void BarometerHandler::getRosParams()
{
}

void BarometerHandler::registerPublishers()
{
  bar_pub_ = nh_.advertise<BarMsg>("air_pressure", 1);
}

void BarometerHandler::registerSubscribers()
{
  event_sub_ = nh_.subscribe("event", 1, &BarometerHandler::eventCb, this);
}

void BarometerHandler::eventCb(const tobas_msgs::Event& event)
{
  switch (event.data)
  {
    case tobas_msgs::Event::SHUTDOWN:
      ros::shutdown();
      break;
    default:
      break;
  }
}
}  // namespace tobas_real
