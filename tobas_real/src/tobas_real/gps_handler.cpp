#include <dh_std_tools/math.hpp>
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
  gps_.disableAllNavMsgs();
  gps_.enableNavMsg(Ublox::NAV_STATUS);
  gps_.enableNavMsg(Ublox::NAV_PVT);
  gps_.enableNavMsg(Ublox::NAV_COV);

  if (!gps_.testConnection())
  {
    rosthrow("Failed to connect to GPS.");
  }

  if (gps_.configureSolutionRate(kMeasurementRate) < 0)
  {
    rosthrow("Failed to set measurement rate.");
  }

  getRosParams();

  gps_msg_.position_covariance_type = GpsMsg::COVARIANCE_TYPE_KNOWN;

  registerPublishers();
  registerSubscribers();
}

void GpsHandler::run()
{
  dh_ros::Stopwatch stopwatch;

  while (ros::ok())
  {
    const ros::Time now = ros::Time::now();
    // stopwatch.start();
    const auto msg_id = gps_.update();
    // stopwatch.stop();
    cout << "Message ID: " << msg_id << endl;

    switch (msg_id)
    {
      case Ublox::NAV_STATUS:
      {
        gps_.decode(status_);
        gps_fix_ok_ = status_.flags & 1;  // p.288, Bitfield flags

        if (!gps_fix_ok_)
        {
          rosErrorThrottle(
            kErrorPeriod, "GPS fix not acquired. Please check GNSS signal strength and receiver's "
                          "connection.");
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
