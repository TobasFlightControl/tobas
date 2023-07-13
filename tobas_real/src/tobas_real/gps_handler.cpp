#include <dh_std_tools/math.hpp>
#include <dh_ros_tools/rosparam.hpp>
#include <dh_ros_tools/exception.hpp>
#include <dh_ros_tools/console_message.hpp>

#include "../../include/tobas_real/gps_handler.hpp"

using namespace std;

namespace tobas_real
{
GpsHandler::GpsHandler() : super(), cov_received_(false)
{
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
  while (ros::ok())
  {
    const ros::Time now = ros::Time::now();

    if (gps_.decodeSingleMessage(Ublox::NAV_COV, data_) == 1)
    {
      if (!cov_received_)
      {
        cov_received_ = true;
      }

      // Update GPS position covariance
      gps_msg_.position_covariance[0] = data_[0];  // NN
      gps_msg_.position_covariance[1] = data_[1];  // NE
      gps_msg_.position_covariance[2] = data_[2];  // ND
      gps_msg_.position_covariance[3] = data_[1];  // EN
      gps_msg_.position_covariance[4] = data_[3];  // EE
      gps_msg_.position_covariance[5] = data_[4];  // ED
      gps_msg_.position_covariance[6] = data_[2];  // DN
      gps_msg_.position_covariance[7] = data_[4];  // DE
      gps_msg_.position_covariance[8] = data_[5];  // DD

      // Update GPS velocity covariance
      vel_msg_.covariance[0] = data_[6];   // NN
      vel_msg_.covariance[1] = data_[7];   // NE
      vel_msg_.covariance[2] = data_[8];   // ND
      vel_msg_.covariance[3] = data_[7];   // EN
      vel_msg_.covariance[4] = data_[9];   // EE
      vel_msg_.covariance[5] = data_[10];  // ED
      vel_msg_.covariance[6] = data_[8];   // DN
      vel_msg_.covariance[7] = data_[10];  // DE
      vel_msg_.covariance[8] = data_[11];  // DD
    }

    if (gps_.decodeSingleMessage(Ublox::NAV_POSLLH, data_) == 1)
    {
      if (!cov_received_)
      {
        return;
      }

      // Update GPS position message
      gps_msg_.header.stamp = now;
      gps_msg_.latitude = data_[2] * 1e-7;   // Latitude [deg]
      gps_msg_.longitude = data_[1] * 1e-7;  // Longitude [deg]
      gps_msg_.altitude = data_[4] * 1e-3;   // Height above mean sea level [m]

      // Publish message
      gps_pub_.publish(gps_msg_);
    }

    if (gps_.decodeSingleMessage(Ublox::NAV_VELNED, data_) == 1)
    {
      if (!cov_received_)
      {
        return;
      }

      // Update GPS velocity message
      vel_msg_.header.stamp = now;
      vel_msg_.vel.x(data_[0] * 1e-2);   // North velocity [m]
      vel_msg_.vel.y(-data_[1] * 1e-2);  // West velocity [m]
      vel_msg_.vel.z(-data_[2] * 1e-2);  // Up velocity [m]

      // Publish message
      vel_pub_.publish(vel_msg_);
    }

    // if (gps_.decodeSingleMessage(Ublox::NAV_PVT, data_) == 1)
    // {
    //   if (!cov_received_)
    //   {
    //     return;
    //   }

    //   // Update GPS position message
    //   gps_msg_.header.stamp = now;
    //   gps_msg_.latitude = data_[1] * 1e-7;   // Latitude [deg]
    //   gps_msg_.longitude = data_[0] * 1e-7;  // Longitude [deg]
    //   gps_msg_.altitude = data_[2] * 1e-3;   // Height above ellipsoid [m]

    //   // Update GPS velocity message
    //   vel_msg_.header.stamp = now;
    //   vel_msg_.vel.x(data_[3] * 1e-3);   // North velocity [m]
    //   vel_msg_.vel.y(-data_[4] * 1e-3);  // West velocity [m]
    //   vel_msg_.vel.z(-data_[5] * 1e-3);  // Up velocity [m]

    //   // Publish messages
    //   gps_pub_.publish(gps_msg_);
    //   vel_pub_.publish(vel_msg_);
    // }

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

void GpsHandler::eventCb(const tobas_msgs::Event& event)
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
