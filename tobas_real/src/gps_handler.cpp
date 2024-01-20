#include <tobas_std_tools/math.hpp>
#include <tobas_std_tools/time.hpp>
#include <tobas_ros_tools/exception.hpp>
#include <tobas_ros_tools/console_message.hpp>

#include <tobas_tools/constants.hpp>
#include <tobas_msgs/Gps.h>

#include "../include/tobas_real/gps_handler.hpp"
#include "../include/tobas_real/common.hpp"

using namespace std;

namespace tobas_real
{
GpsHandler::GpsHandler(const ros::NodeHandle& nh, const ros::NodeHandle& pnh, const string& name)
  : super(nh, pnh, name)
{
  getRosParams();
  configureGnssReceiver();

  registerPublishers();
  registerSubscribers();

  main_timer_ = nh_.createTimer(kMainTimerRate, &self::mainTimerCb, this);
}

void GpsHandler::getRosParams()
{
}

void GpsHandler::registerPublishers()
{
  gps_pub_ = nh_.advertise<tobas_msgs::Gps>(tobas::kGpsTopic, 1);
}

void GpsHandler::registerSubscribers()
{
  super::registerSubscribers();
}

void GpsHandler::configureGnssReceiver()
{
  if (!gps_.enableAllMsgs(false))
    ROS_THROW_NAMED(name_, "Failed to disable all navigation messsages.");
  if (!gps_.enableMsg(Ublox::NAV_PVT, true))
    ROS_THROW_NAMED(name_, "Failed to enable NAV_PVT");
  if (!gps_.enableMsg(Ublox::NAV_COV, true))
    ROS_THROW_NAMED(name_, "Failed to enable NAV_COV");

  if (!gps_.configureSolutionRate(kMeasurementRate))
    ROS_THROW_NAMED(name_, "Failed to set measurement rate.");

  if (!gps_.configureDynamicsModel(Ublox::AIRBORNE_2G))
    ROS_THROW_NAMED(name_, "Failed to set dynamics model.");

  // データシートを見るに複数のメインGNSSを組み合わせると処理が重くなるから，GPSだけで良さそう
  // https://www.u-blox.com/en/product/neo-m8-series
  if (!gps_.configureGnss_GPS(true))
    ROS_THROW_NAMED(name_, "Failed to configure GPS.");
  if (!gps_.configureGnss_SBAS(true))
    ROS_THROW_NAMED(name_, "Failed to configure SBAS.");
  if (!gps_.configureGnss_Galileo(false))
    ROS_THROW_NAMED(name_, "Failed to configure Galileo.");
  if (!gps_.configureGnss_BeiDou(false))
    ROS_THROW_NAMED(name_, "Failed to configure BeiDou.");
  if (!gps_.configureGnss_QZSS(true))
    ROS_THROW_NAMED(name_, "Failed to configure QZSS.");
  if (!gps_.configureGnss_GLONASS(false))
    ROS_THROW_NAMED(name_, "Failed to configure GLONASS.");
}

void GpsHandler::eventCb(const tobas_msgs::EventConstPtr& event)
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

void GpsHandler::mainTimerCb(const ros::TimerEvent& event)
{
  const auto msg = gps_.update();
  // cout << "Message ID: " << msg << endl;

  switch (msg)
  {
    case Ublox::NAV_PVT:
    {
      if (!cov_received_)
      {
        break;
      }

      gps_.decode(pvt_);
      // cout << pvt_ << endl;

      // const auto gps_tp = tobas_std::timePointFromUTC(
      //   pvt_.year, pvt_.month, pvt_.day, pvt_.hour, pvt_.min, pvt_.sec, pvt_.nano);
      // const auto cur_tp = chrono::system_clock::now();  // UTCを得るにはインターネットが必要
      // const auto gps_delay = chrono::duration_cast<chrono::milliseconds>(cur_tp - gps_tp);
      // rosInfo(name_, "GPS delay: " << gps_delay.count() << "[ms]");

      if (!pvt_.gnssFixOk || pvt_.fixType != Ublox::FIX_3D)
      {
        rosWarnThrottle(
          kWarnPeriod, name_, "GPS no fix. Please check GNSS signal and connection strength.");
        break;
      }

      // Create GPS message
      const auto gps_msg = boost::make_shared<tobas_msgs::Gps>();
      gps_msg->header.stamp = event.current_real;
      gps_msg->header.frame_id = "gps_frame";

      // Fill GPS position
      gps_msg->latitude = pvt_.lat;                     // Latitude [deg]
      gps_msg->longitude = pvt_.lon;                    // Longitude [deg]
      gps_msg->altitude = pvt_.hMSL;                    // Height above ellipsoid [m]
      gps_msg->position_covariance[0] = cov_.posCovNN;  // NN
      gps_msg->position_covariance[1] = cov_.posCovNE;  // NE
      gps_msg->position_covariance[2] = cov_.posCovND;  // ND
      gps_msg->position_covariance[3] = cov_.posCovNE;  // EN
      gps_msg->position_covariance[4] = cov_.posCovEE;  // EE
      gps_msg->position_covariance[5] = cov_.posCovED;  // ED
      gps_msg->position_covariance[6] = cov_.posCovND;  // DN
      gps_msg->position_covariance[7] = cov_.posCovED;  // DE
      gps_msg->position_covariance[8] = cov_.posCovDD;  // DD

      // Fill GPS velocity
      gps_msg->header.stamp = event.current_real;
      gps_msg->ground_speed.x(pvt_.velN);               // North velocity [m/s]
      gps_msg->ground_speed.y(-pvt_.velE);              // West velocity [m/s]
      gps_msg->ground_speed.z(-pvt_.velD);              // Up velocity [m/s]
      gps_msg->velocity_covariance[0] = cov_.velCovNN;  // NN
      gps_msg->velocity_covariance[1] = cov_.velCovNE;  // NE
      gps_msg->velocity_covariance[2] = cov_.velCovND;  // ND
      gps_msg->velocity_covariance[3] = cov_.velCovNE;  // EN
      gps_msg->velocity_covariance[4] = cov_.velCovEE;  // EE
      gps_msg->velocity_covariance[5] = cov_.velCovED;  // ED
      gps_msg->velocity_covariance[6] = cov_.velCovND;  // DN
      gps_msg->velocity_covariance[7] = cov_.velCovED;  // DE
      gps_msg->velocity_covariance[8] = cov_.velCovDD;  // DD

      // Publish GPS message
      gps_pub_.publish(gps_msg);

      break;
    }
    case Ublox::NAV_COV:
    {
      gps_.decode(cov_);

      if (!cov_received_)
      {
        cov_received_ = true;
      }

      break;
    }
    default:
    {
      rosWarn(name_, "Unnecessary UBX message: " << msg);
      break;
    }
  }
}
}  // namespace tobas_real
