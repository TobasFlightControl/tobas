#include <tobas_math/core.hpp>
#include <tobas_std_tools/time.hpp>
#include <tobas_tools/constants.hpp>
#include <tobas_std_tools/console.hpp>
#include <tobas_msgs/Gps.h>

#include "../include/tobas_navio_ros/gps_handler.hpp"
#include "../include/tobas_navio_ros/common.hpp"

using namespace std;

namespace tobas_navio_ros
{
GpsHandler::GpsHandler(ros::NodeHandle& nh, ros::NodeHandle& pnh, const string& name) : super(nh, pnh, name)
{
  if (!gps_.initialize())
    TOBAS_EXIT("Failed to initialize GNSS receiver.");

  try
  {
    configureGnssReceiver();
  }
  catch (const runtime_error& e)
  {
    TOBAS_EXIT("Failed to configure GNSS receiver: ", e.what());
  }

  gps_pub_ = nh_.advertise<tobas_msgs::Gps>(tobas::kGpsTopic, 1);

  // Start main timer with maximum rate
  main_timer_ = nh_.createTimer(ros::Duration(0), &self::mainTimerCb, this);
}

void GpsHandler::configureGnssReceiver()
{
  // 設定をデフォルトに戻す
  gps_.clearConfigurations();
  gps_.loadConfigurations();

  // gps_.enableAllMsgs(false);
  gps_.enableMsg(navio::NEOM8N::NAV_PVT, true);
  gps_.enableMsg(navio::NEOM8N::NAV_COV, true);

  gps_.configureSolutionRate(kMeasurementRate);
  gps_.configureDynamicsModel(navio::NEOM8N::AIRBORNE_2G);
  gps_.configurePowerMode(navio::NEOM8N::FULL_POWER);

  // データシートを見るに複数のメインGNSSを組み合わせると処理が重くなるから，GPSだけで良さそう
  // https://www.u-blox.com/en/product/neo-m8-series
  gps_.configureGnss_GPS(true);
  gps_.configureGnss_SBAS(true);
  gps_.configureGnss_Galileo(false);
  gps_.configureGnss_BeiDou(false);
  gps_.configureGnss_QZSS(true);
  gps_.configureGnss_GLONASS(false);
}

void GpsHandler::mainTimerCb(const ros::TimerEvent& event)
{
  const auto msg = gps_.update();
  // cout << "Message ID: " << msg << endl;

  switch (msg)
  {
    case navio::NEOM8N::NAV_PVT:
    {
      if (!cov_received_)
        break;

      gps_.decode(pvt_);

      // Create GPS message
      const auto gps_msg = boost::make_shared<tobas_msgs::Gps>();
      gps_msg->header.stamp = event.current_real;

      // Fill fix type
      gps_msg->fix_type = pvt_.fixType;

      // Fill GPS position
      gps_msg->latitude = pvt_.lat;                        // Latitude [deg]
      gps_msg->longitude = pvt_.lon;                       // Longitude [deg]
      gps_msg->altitude = pvt_.hMSL;                       // Height above ellipsoid [m]
      gps_msg->position_covariance(0, 0) = cov_.posCovNN;  // NN
      gps_msg->position_covariance(0, 1) = cov_.posCovNE;  // NE
      gps_msg->position_covariance(0, 2) = cov_.posCovND;  // ND
      gps_msg->position_covariance(1, 0) = cov_.posCovNE;  // EN
      gps_msg->position_covariance(1, 1) = cov_.posCovEE;  // EE
      gps_msg->position_covariance(1, 2) = cov_.posCovED;  // ED
      gps_msg->position_covariance(2, 0) = cov_.posCovND;  // DN
      gps_msg->position_covariance(2, 1) = cov_.posCovED;  // DE
      gps_msg->position_covariance(2, 2) = cov_.posCovDD;  // DD

      // Fill GPS velocity
      gps_msg->ground_speed.x(pvt_.velN);                  // North velocity [m/s]
      gps_msg->ground_speed.y(-pvt_.velE);                 // West velocity [m/s]
      gps_msg->ground_speed.z(-pvt_.velD);                 // Up velocity [m/s]
      gps_msg->velocity_covariance(0, 0) = cov_.velCovNN;  // NN
      gps_msg->velocity_covariance(0, 1) = cov_.velCovNE;  // NE
      gps_msg->velocity_covariance(0, 2) = cov_.velCovND;  // ND
      gps_msg->velocity_covariance(1, 0) = cov_.velCovNE;  // EN
      gps_msg->velocity_covariance(1, 1) = cov_.velCovEE;  // EE
      gps_msg->velocity_covariance(1, 2) = cov_.velCovED;  // ED
      gps_msg->velocity_covariance(2, 0) = cov_.velCovND;  // DN
      gps_msg->velocity_covariance(2, 1) = cov_.velCovED;  // DE
      gps_msg->velocity_covariance(2, 2) = cov_.velCovDD;  // DD

      // Publish GPS message
      gps_pub_.publish(gps_msg);

      break;
    }
    case navio::NEOM8N::NAV_COV:
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
