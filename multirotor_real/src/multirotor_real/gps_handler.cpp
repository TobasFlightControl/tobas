#include <dh_std_tools/math.hpp>
#include <dh_ros_tools/rosparam.hpp>
#include <dh_ros_tools/exception.hpp>
#include <dh_ros_tools/console_message.hpp>

#include "../../include/multirotor_real/gps_handler.hpp"

#define SOLUTION_RATE 1e+3
#define TIMER_PERIOD 2e-4

using namespace std;

GpsHandler::GpsHandler(ros::NodeHandle& nh) : cov_received_(false)
{
  if (!gps_.testConnection())
  {
    throw dh_ros::RuntimeError("Failed to connect to GPS.");
  }

  if (!gps_.configureSolutionRate(SOLUTION_RATE))
  {
    throw dh_ros::RuntimeError("Failed to set solution rate.");
  }

  string drone_name = dh_ros::getParam<string>("/drone_name");
  gps_pub_ = nh.advertise<GpsMsg>("/" + drone_name + "/gps", 1);
  vel_pub_ = nh.advertise<VelMsg>("/" + drone_name + "/ground_speed", 1);

  timer_ = nh.createTimer(ros::Duration(TIMER_PERIOD), &GpsHandler::timerCb, this);
}

void GpsHandler::timerCb(const ros::TimerEvent&)
{
  if (gps_.decodeSingleMessage(Ublox::NAV_COV, data_) == 1)
  {
    cov_received_ = true;

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
    vel_msg_.vel.covariance[0] = data_[6];  // NN
    vel_msg_.vel.covariance[1] = data_[7];  // NE
    vel_msg_.vel.covariance[2] = data_[8];  // ND
    vel_msg_.vel.covariance[3] = data_[7];  // EN
    vel_msg_.vel.covariance[4] = data_[9];  // EE
    vel_msg_.vel.covariance[5] = data_[10];  // ED
    vel_msg_.vel.covariance[6] = data_[8];  // DN
    vel_msg_.vel.covariance[7] = data_[10];  // DE
    vel_msg_.vel.covariance[8] = data_[11];  // DD
  }

  if (gps_.decodeSingleMessage(Ublox::NAV_PVT, data_) == 1)
  {
    if (!cov_received_)
    {
      return;
    }

    ros::Time now = ros::Time::now();

    // Update GPS position message
    gps_msg_.header.stamp = now;
    gps_msg_.latitude = data_[1] * 1e-7;   // Latitude [deg]
    gps_msg_.longitude = data_[0] * 1e-7;  // Longitude [deg]
    gps_msg_.altitude = data_[2] * 1e-3;   // Height above ellipsoid [m]

    // Update GPS velocity message
    vel_msg_.header.stamp = now;
    vel_msg_.vel.vel.vx = data_[3] * 1e-3;   // North velocity [m]
    vel_msg_.vel.vel.vy = -data_[4] * 1e-3;  // West velocity [m]
    vel_msg_.vel.vel.vz = -data_[5] * 1e-3;  // Up velocity [m]

    // Publish messages
    gps_pub_.publish(gps_msg_);
    vel_pub_.publish(vel_msg_);
  }
}
