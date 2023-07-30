#include <dh_std_tools/math.hpp>
#include <dh_std_tools/time.hpp>
#include <dh_ros_tools/rosparam.hpp>
#include <dh_ros_tools/exception.hpp>
#include <dh_ros_tools/console_message.hpp>
#include <dh_ros_tools/stopwatch.hpp>

#include "../../include/tobas_real/gps_handler.hpp"
#include "../../include/tobas_real/common.hpp"

using namespace std;

namespace tobas_real
{
GpsHandler::GpsHandler() : super(), gps_fix_ok_(false), cov_received_(false)
{
  getRosParams();
  configureGnssReceiver();

  gps_msg_.position_covariance_type = GpsMsg::COVARIANCE_TYPE_KNOWN;

  registerPublishers();
  registerSubscribers();
}

void GpsHandler::run()
{
  dh_ros::Stopwatch stopwatch;

  while (ros::ok())
  {
    // stopwatch.start();
    const auto msg_id = gps_.update();
    // cout << "Message ID: " << msg_id << endl;
    // stopwatch.stop();

    const ros::Time now = ros::Time::now();

    switch (msg_id)
    {
      case Ublox::NAV_STATUS:
      {
        gps_.decode(status_);
        gps_fix_ok_ = status_.flags & 1;  // p.288, Bitfield flags

        if (!gps_fix_ok_)
        {
          rosErrorThrottle(
            kErrorPeriod, "GPS no fix. Please check GNSS signal and connection strength.");
        }

        break;
      }
      case Ublox::NAV_PVT:
      {
        if (!isReadyToPublish())
        {
          continue;
        }

        gps_.decode(pvt_);
        // cout << pvt_ << endl;

        // auto gps_tm =
        //   dh_std::tmFromUTC(pvt_.year, pvt_.month, pvt_.day, pvt_.hour, pvt_.min, pvt_.sec);
        // const auto gps_tp = dh_std::tmToTimePoint(gps_tm);
        // const auto cur_tp = chrono::system_clock::now();  // インターネット接続が必要
        // const auto gps_delay = chrono::duration_cast<chrono::milliseconds>(cur_tp - gps_tp);
        // cout << "GPS time:" << endl << gps_tm << endl;
        // cout << "Current time:" << endl << dh_std::timePointToTm(cur_tp) << endl;
        // rosInfo("GPS delay: " << gps_delay.count() << "[ms]");

        // Update GPS position message
        gps_msg_.header.stamp = now;
        gps_msg_.latitude = pvt_.lat;   // Latitude [deg]
        gps_msg_.longitude = pvt_.lon;  // Longitude [deg]
        gps_msg_.altitude = pvt_.hMSL;  // Height above ellipsoid [m]

        // Update GPS velocity message
        vel_msg_.header.stamp = now;
        vel_msg_.vel.x(pvt_.velN);   // North velocity [m]
        vel_msg_.vel.y(-pvt_.velE);  // West velocity [m]
        vel_msg_.vel.z(-pvt_.velD);  // Up velocity [m]

        // Publish messages
        gps_pub_.publish(gps_msg_);
        vel_pub_.publish(vel_msg_);

        break;
      }
      case Ublox::NAV_COV:
      {
        if (!cov_received_)
        {
          cov_received_ = true;
        }

        gps_.decode(cov_);

        // Update GPS position covariance
        gps_msg_.position_covariance[0] = cov_.posCovNN;  // NN
        gps_msg_.position_covariance[1] = cov_.posCovNE;  // NE
        gps_msg_.position_covariance[2] = cov_.posCovND;  // ND
        gps_msg_.position_covariance[3] = cov_.posCovNE;  // EN
        gps_msg_.position_covariance[4] = cov_.posCovEE;  // EE
        gps_msg_.position_covariance[5] = cov_.posCovED;  // ED
        gps_msg_.position_covariance[6] = cov_.posCovND;  // DN
        gps_msg_.position_covariance[7] = cov_.posCovED;  // DE
        gps_msg_.position_covariance[8] = cov_.posCovDD;  // DD

        // Update GPS velocity covariance
        vel_msg_.covariance[0] = cov_.velCovNN;  // NN
        vel_msg_.covariance[1] = cov_.velCovNE;  // NE
        vel_msg_.covariance[2] = cov_.velCovND;  // ND
        vel_msg_.covariance[3] = cov_.velCovNE;  // EN
        vel_msg_.covariance[4] = cov_.velCovEE;  // EE
        vel_msg_.covariance[5] = cov_.velCovED;  // ED
        vel_msg_.covariance[6] = cov_.velCovND;  // DN
        vel_msg_.covariance[7] = cov_.velCovED;  // DE
        vel_msg_.covariance[8] = cov_.velCovDD;  // DD

        break;
      }
      default:
      {
        rosWarn("Unnecessary UBX NAV message: " << msg_id);
        break;
      }
    }

    ros::spinOnce();
    usleep(kSleepTime);
  }
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
  event_sub_ = nh_.subscribe("event", 1, &GpsHandler::eventCb, this);
}

void GpsHandler::configureGnssReceiver()
{
  if (!gps_.enableAllNavMsgs(false))
  {
    rosthrow("Failed to disable all navigation messsages.");
  }
  if (!gps_.enableNavMsg(Ublox::NAV_STATUS, true))
  {
    rosthrow("Failed to enable NAV_STATUS");
  }
  if (!gps_.enableNavMsg(Ublox::NAV_PVT, true))
  {
    rosthrow("Failed to enable NAV_PVT");
  }
  if (!gps_.enableNavMsg(Ublox::NAV_COV, true))
  {
    rosthrow("Failed to enable NAV_COV");
  }

  if (!gps_.configureSolutionRate(kMeasurementRate))
    rosthrow("Failed to set measurement rate.");

  if (!gps_.configureDynamicsModel(Ublox::AIRBORNE_2G))
    rosthrow("Failed to set dynamics model.");

  // データシートを見るに複数のメインGNSSを組み合わせると処理が重くなるから，GPSだけで良さそう
  // https://www.u-blox.com/en/product/neo-m8-series
  if (!gps_.configureGnss_GPS(true))
  {
    rosthrow("Failed to configure GPS.");
  }
  if (!gps_.configureGnss_SBAS(true))
  {
    rosthrow("Failed to configure SBAS.");
  }
  if (!gps_.configureGnss_Galileo(false))
  {
    rosthrow("Failed to configure Galileo.");
  }
  if (!gps_.configureGnss_BeiDou(false))
  {
    rosthrow("Failed to configure BeiDou.");
  }
  if (!gps_.configureGnss_QZSS(true))
  {
    rosthrow("Failed to configure QZSS.");
  }
  if (!gps_.configureGnss_GLONASS(false))
  {
    rosthrow("Failed to configure GLONASS.");
  }
}

bool GpsHandler::isReadyToPublish() const
{
  return gps_fix_ok_ && cov_received_;
}

void GpsHandler::eventCb(const tobas_msgs::Event& event)
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
