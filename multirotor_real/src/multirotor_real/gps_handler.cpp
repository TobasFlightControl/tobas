#include <dh_std_tools/math.hpp>
#include <dh_ros_tools/rosparam.hpp>
#include <dh_ros_tools/exception.hpp>
#include <dh_ros_tools/console_message.hpp>

#include "../../include/multirotor_real/gps_handler.hpp"

#define SOLUTION_RATE 1e+3
#define TIMER_PERIOD 2e-4

#define LATITUDE_IDX 2
#define LONGITUDE_IDX 1
#define ALTITUDE_IDX 4
#define HOR_POS_ACCURACY_IDX 5
#define VER_POS_ACCURACY_IDX 6

// U-blox NEO-M8
// https://content.u-blox.com/sites/default/files/NEO-M8_DataSheet_%28UBX-13003366%29.pdf
#define HOR_POS_ACCURACY 2.5  // [m]
#define VEL_ACCURACY 0.05     // [m/s]

using namespace std;

GpsHandler::GpsHandler(ros::NodeHandle& nh)
{
  if (!gps_.testConnection())
  {
    throw dh_ros::RuntimeError("Failed to connect to GPS.");
  }

  if (!gps_.configureSolutionRate(SOLUTION_RATE))
  {
    dh_ros::rosError("Failed to set solution rate.");
  }

  string drone_name = dh_ros::getParam<string>("/drone_name");
  gps_pub_ = nh.advertise<GpsMsg>("/" + drone_name + "/gps", 1);

  timer_ = nh.createTimer(ros::Duration(TIMER_PERIOD), &GpsHandler::timerCb, this);
}

void GpsHandler::timerCb(const ros::TimerEvent&)
{
  if (gps_.decodeSingleMessage(Ublox::NAV_POSLLH, gps_data_) != 1)
  {
    return;
  }

  // Update GPS message
  gps_msg_.header.stamp = ros::Time::now();

  gps_msg_.latitude = gps_data_[LATITUDE_IDX] / 10000000;
  gps_msg_.longitude = gps_data_[LONGITUDE_IDX] / 10000000;
  gps_msg_.altitude = gps_data_[ALTITUDE_IDX] / 1000;  // Height above mean sea level

  // FIXME: GPSの精度に関する情報が無かったため，正確度そのまま精度(標準偏差)として用いている
  double hor_pos_accuracy = gps_data_[HOR_POS_ACCURACY_IDX] / 1000;
  double ver_pos_accuracy = gps_data_[VER_POS_ACCURACY_IDX] / 1000;
  gps_msg_.position_covariance[0] = dh_std::sqr(hor_pos_accuracy);
  gps_msg_.position_covariance[4] = dh_std::sqr(hor_pos_accuracy);
  gps_msg_.position_covariance[8] = dh_std::sqr(ver_pos_accuracy);

  // Publish GPS Message
  gps_pub_.publish(gps_msg_);
}
