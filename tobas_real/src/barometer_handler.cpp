#include <boost/property_tree/ini_parser.hpp>

#include <dh_std_tools/math.hpp>
#include <dh_ros_tools/rosparam.hpp>
#include <dh_ros_tools/rate.hpp>

#include "../include/tobas_real/barometer_handler.hpp"
#include "../include/tobas_real/common.hpp"

namespace tobas_real
{
BarometerHandler::BarometerHandler() : super()
{
  getRosParams();

  barometer_.initialize();
  if (!barometer_.testConnection())
  {
    rosthrow("Barometer test failed.");
  }

  bar_msg_.header.frame_id = "barometer_frame";
  bar_msg_.variance = dh_std::sqr(pressure_noise_density_) * kUpdateRate;  // [Pa^2]

  registerPublishers();
  registerSubscribers();
}

void BarometerHandler::run()
{
  dh_ros::Rate rate(kUpdateRate);

  while (ros::ok())
  {
    barometer_.refreshPressure();
    usleep(kWaitToUpdateSensor);  // この待ち時間が必須
    barometer_.readPressure();
    barometer_.calculatePressureAndTemperature();

    const auto pressure = barometer_.getPressure() * 100;  // mbar -> Pa
    if (pressure < kMinAirPressure || kMaxAirPressure < pressure)
    {
      rosError("Strange air pressure: " << pressure << " [Pa]");
      continue;
    }

    bar_msg_.fluid_pressure = pressure;
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

void BarometerHandler::readConfig()
{
  boost::property_tree::ptree pt;
  boost::property_tree::ini_parser::read_ini(kConfigPath, pt);

  pressure_noise_density_ = pt.get<double>(kConfigKey_PressureNoiseDensity);
}

void BarometerHandler::eventCb(const tobas_msgs::Event& event)
{
  switch (event.data)
  {
    case tobas_msgs::Event::SHUTDOWN:
      // ros::shutdown();
      break;
    default:
      break;
  }
}
}  // namespace tobas_real
