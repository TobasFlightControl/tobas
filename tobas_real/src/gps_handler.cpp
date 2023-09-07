#include <dh_std_tools/math.hpp>
#include <dh_std_tools/time.hpp>
#include <dh_ros_tools/rosparam.hpp>
#include <dh_ros_tools/exception.hpp>
#include <dh_ros_tools/console_message.hpp>
#include <dh_ros_tools/stopwatch.hpp>

#include "../include/tobas_real/gps_handler.hpp"
#include "../include/tobas_real/common.hpp"

using namespace std;

namespace tobas_real
{
GpsHandler::GpsHandler(ros::NodeHandle nh, ros::NodeHandle pnh, string name) : super(nh, pnh, name)
{
  getRosParams();
  configureGnssReceiver();

  registerPublishers();
  registerSubscribers();

  // Create first one-shot timer
  main_timer_ = nh_.createTimer(ros::Duration(kSleepTime), &GpsHandler::mainTimerCb, this, true);
}

void GpsHandler::getRosParams()
{
}

void GpsHandler::registerPublishers()
{
  gps_pub_ = nh_.advertise<GpsMsg>("gps", 1);
  vel_pub_ = nh_.advertise<VelMsg>("ground_speed", 1);
}

void GpsHandler::registerSubscribers()
{
  event_sub_ = nh_.subscribe("event", 1, &GpsHandler::eventCb, this, tcpNoDelay());
}

void GpsHandler::configureGnssReceiver()
{
  if (!gps_.enableAllMsgs(false))
    rosthrow(name_, "Failed to disable all navigation messsages.");

  if (!gps_.enableMsg(Ublox::NAV_PVT, true))
    rosthrow(name_, "Failed to enable NAV_PVT");

  if (!gps_.enableMsg(Ublox::NAV_COV, true))
    rosthrow(name_, "Failed to enable NAV_COV");

  if (!gps_.enableMsg(Ublox::NAV_TIMEUTC, true))
    rosthrow(name_, "Failed to enable NAV_TIMEUTC");

  if (!gps_.configureSolutionRate(kMeasurementRate))
    rosthrow(name_, "Failed to set measurement rate.");

  if (!gps_.configureDynamicsModel(Ublox::AIRBORNE_2G))
    rosthrow(name_, "Failed to set dynamics model.");

  // データシートを見るに複数のメインGNSSを組み合わせると処理が重くなるから，GPSだけで良さそう
  // https://www.u-blox.com/en/product/neo-m8-series
  if (!gps_.configureGnss_GPS(true))
    rosthrow(name_, "Failed to configure GPS.");

  if (!gps_.configureGnss_SBAS(true))
    rosthrow(name_, "Failed to configure SBAS.");

  if (!gps_.configureGnss_Galileo(false))
    rosthrow(name_, "Failed to configure Galileo.");

  if (!gps_.configureGnss_BeiDou(false))
    rosthrow(name_, "Failed to configure BeiDou.");

  if (!gps_.configureGnss_QZSS(true))
    rosthrow(name_, "Failed to configure QZSS.");

  if (!gps_.configureGnss_GLONASS(false))
    rosthrow(name_, "Failed to configure GLONASS.");
}

void GpsHandler::eventCb(const tobas_msgs::EventConstPtr& event)
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

      // const auto gps_tp = dh_std::timePointFromUTC(
      //   pvt_.year, pvt_.month, pvt_.day, pvt_.hour, pvt_.min, pvt_.sec, pvt_.nano);
      // const auto cur_tp = chrono::system_clock::now();  // UTCを得るにはインターネットが必要
      // const auto gps_delay = chrono::duration_cast<chrono::milliseconds>(cur_tp - gps_tp);
      // rosInfo(name_, "GPS delay: " << gps_delay.count() << "[ms]");

      if (!pvt_.gnssFixOk || pvt_.fixType != Ublox::FIX_3D)
      {
        rosWarnThrottle(
          kErrorPeriod, name_, "GPS no fix. Please check GNSS signal and connection strength.");
        break;
      }

      // Create GPS position message
      const auto gps_msg = boost::make_shared<GpsMsg>();
      gps_msg->header.stamp = event.current_real;
      gps_msg->header.frame_id = "gps_frame";
      gps_msg->position_covariance_type = GpsMsg::COVARIANCE_TYPE_KNOWN;
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

      // Create GPS velocity message
      const auto vel_msg = boost::make_shared<VelMsg>();
      vel_msg->header.stamp = event.current_real;
      vel_msg->vel.x(pvt_.velN);               // North velocity [m]
      vel_msg->vel.y(-pvt_.velE);              // West velocity [m]
      vel_msg->vel.z(-pvt_.velD);              // Up velocity [m]
      vel_msg->covariance[0] = cov_.velCovNN;  // NN
      vel_msg->covariance[1] = cov_.velCovNE;  // NE
      vel_msg->covariance[2] = cov_.velCovND;  // ND
      vel_msg->covariance[3] = cov_.velCovNE;  // EN
      vel_msg->covariance[4] = cov_.velCovEE;  // EE
      vel_msg->covariance[5] = cov_.velCovED;  // ED
      vel_msg->covariance[6] = cov_.velCovND;  // DN
      vel_msg->covariance[7] = cov_.velCovED;  // DE
      vel_msg->covariance[8] = cov_.velCovDD;  // DD

      // Publish messages
      gps_pub_.publish(gps_msg);
      vel_pub_.publish(vel_msg);

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
    case Ublox::NAV_TIMEUTC:
    {
      gps_.decode(timeutc_);

      const auto gps_tp = dh_std::timePointFromUTC(
        timeutc_.year, timeutc_.month, timeutc_.day, timeutc_.hour, timeutc_.min, timeutc_.sec,
        timeutc_.nano);
      const auto cur_tp = chrono::system_clock::now();  // UTCを得るにはインターネットが必要
      const auto gps_delay = chrono::duration_cast<chrono::milliseconds>(cur_tp - gps_tp);
      rosInfo(name_, "TIMEUTC delay: " << gps_delay.count() << "[ms]");
    }
    default:
    {
      rosWarn(name_, "Unnecessary UBX message: " << msg);
      break;
    }
  }

  // Update one-shot timer
  main_timer_ = nh_.createTimer(ros::Duration(kSleepTime), &GpsHandler::mainTimerCb, this, true);
}
}  // namespace tobas_real
