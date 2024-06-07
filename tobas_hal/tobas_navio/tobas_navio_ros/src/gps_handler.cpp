#include <tobas_std_tools/math.hpp>
#include <tobas_std_tools/time.hpp>
#include <tobas_tools/constants.hpp>
#include <tobas_msgs/Gps.h>

#include "../include/tobas_navio_ros/gps_handler.hpp"
#include "../include/tobas_navio_ros/common.hpp"

using namespace std;

namespace tobas_navio_ros
{
GpsHandler::GpsHandler(const ros::NodeHandle& nh, const ros::NodeHandle& pnh, const string& name) : super(nh, pnh, name)
{
  configureGnssReceiver();

  gps_pub_ = nh_.advertise<tobas_msgs::Gps>(tobas::kGpsTopic, 1);

  // Start main timer with maximum rate
  main_timer_ = nh_.createTimer(ros::Duration(0), &self::mainTimerCb, this);
}

void GpsHandler::configureGnssReceiver()
{
  if (!gps_.enableAllMsgs(false))
    TOBAS_EXIT("Failed to disable all navigation messsages.");
  if (!gps_.enableMsg(navio::Ublox::NAV_PVT, true))
    TOBAS_EXIT("Failed to enable NAV_PVT");
  if (!gps_.enableMsg(navio::Ublox::NAV_COV, true))
    TOBAS_EXIT("Failed to enable NAV_COV");

  if (!gps_.configureSolutionRate(kMeasurementRate))
    TOBAS_EXIT("Failed to set measurement rate.");

  if (!gps_.configureDynamicsModel(navio::Ublox::AIRBORNE_2G))
    TOBAS_EXIT("Failed to set dynamics model.");

  if (!gps_.configurePowerMode(navio::Ublox::FULL_POWER))
    TOBAS_EXIT("Failed to set power mode.");

  // データシートを見るに複数のメインGNSSを組み合わせると処理が重くなるから，GPSだけで良さそう
  // https://www.u-blox.com/en/product/neo-m8-series
  switch (gps_.configureGnss_GPS(true))
  {
    case navio::Ublox::E_NO_ERROR:
      break;
    case navio::Ublox::E_NOT_ACKNOWLEDGED:
      TOBAS_EXIT("The GPS configuration was denied.");
    case navio::Ublox::E_COMMUNICATION_TIMEOUT:
      TOBAS_EXIT("Failed to communicate with the GNSS receiver in the GPS configuration.");
    default:
      throw runtime_error("Unknown error code.");
  }
  switch (gps_.configureGnss_SBAS(true))
  {
    case navio::Ublox::E_NO_ERROR:
      break;
    case navio::Ublox::E_NOT_ACKNOWLEDGED:
      TOBAS_EXIT("The SBAS configuration was denied.");
    case navio::Ublox::E_COMMUNICATION_TIMEOUT:
      TOBAS_EXIT("Failed to communicate with the GNSS receiver in the SBAS configuration.");
    default:
      throw runtime_error("Unknown error code.");
  }
  switch (gps_.configureGnss_Galileo(false))
  {
    case navio::Ublox::E_NO_ERROR:
      break;
    case navio::Ublox::E_NOT_ACKNOWLEDGED:
      TOBAS_EXIT("The Galileo configuration was denied.");
    case navio::Ublox::E_COMMUNICATION_TIMEOUT:
      TOBAS_EXIT("Failed to communicate with the GNSS receiver in the Galileo configuration.");
    default:
      throw runtime_error("Unknown error code.");
  }
  switch (gps_.configureGnss_BeiDou(false))
  {
    case navio::Ublox::E_NO_ERROR:
      break;
    case navio::Ublox::E_NOT_ACKNOWLEDGED:
      TOBAS_EXIT("The BeiDou configuration was denied.");
    case navio::Ublox::E_COMMUNICATION_TIMEOUT:
      TOBAS_EXIT("Failed to communicate with the GNSS receiver in the BeiDou configuration.");
    default:
      throw runtime_error("Unknown error code.");
  }
  switch (gps_.configureGnss_QZSS(true))
  {
    case navio::Ublox::E_NO_ERROR:
      break;
    case navio::Ublox::E_NOT_ACKNOWLEDGED:
      TOBAS_EXIT("The QZSS configuration was denied.");
    case navio::Ublox::E_COMMUNICATION_TIMEOUT:
      TOBAS_EXIT("Failed to communicate with the GNSS receiver in the QZSS configuration.");
    default:
      throw runtime_error("Unknown error code.");
  }
  switch (gps_.configureGnss_GLONASS(false))
  {
    case navio::Ublox::E_NO_ERROR:
      break;
    case navio::Ublox::E_NOT_ACKNOWLEDGED:
      TOBAS_EXIT("The GLONASS configuration was denied.");
    case navio::Ublox::E_COMMUNICATION_TIMEOUT:
      TOBAS_EXIT("Failed to communicate with the GNSS receiver in the GLONASS configuration.");
    default:
      throw runtime_error("Unknown error code.");
  }
}

void GpsHandler::mainTimerCb(const ros::TimerEvent& event)
{
  const auto msg = gps_.update();
  // cout << "Message ID: " << msg << endl;

  switch (msg)
  {
    case navio::Ublox::NAV_PVT:
    {
      if (!cov_received_)
        break;

      gps_.decode(pvt_);
      // cout << pvt_ << endl;

      // Create GPS message
      const auto gps_msg = boost::make_shared<tobas_msgs::Gps>();
      gps_msg->header.stamp = event.current_real;

      // Fill fix type
      gps_msg->fix_type = pvt_.fixType;

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

      // Fill the communication delay
      const auto gps_tp =
        tobas_std::timePointFromUTC(pvt_.year, pvt_.month, pvt_.day, pvt_.hour, pvt_.min, pvt_.sec, pvt_.nano);
      const auto cur_tp = chrono::system_clock::now();  // UTCを得るにはインターネットが必要
      gps_msg->delay.fromNSec(chrono::duration_cast<chrono::nanoseconds>(cur_tp - gps_tp).count());

      // Publish GPS message
      gps_pub_.publish(gps_msg);

      break;
    }
    case navio::Ublox::NAV_COV:
    {
      gps_.decode(cov_);

      if (!cov_received_)
        cov_received_ = true;

      break;
    }
    default:
    {
      TOBAS_WARN("Unnecessary UBX message: ", msg);
      break;
    }
  }
}
}  // namespace tobas_navio_ros
