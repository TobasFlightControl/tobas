#include <dh_std_tools/math.hpp>
#include <dh_ros_tools/rosparam.hpp>

#include "../../include/multirotor_real/gps_handler.hpp"

#define SOLUTION_RATE 1e+3
#define TIMER_PERIOD 2e-4

#define LATITUDE_IDX 2
#define LONGITUDE_IDX 1
#define ALTITUDE_IDX 4
#define HOR_STD_DEV_IDX 5
#define VER_STD_DEV_IDX 6

using namespace std;

GpsHandler::GpsHandler(ros::NodeHandle& nh)
{
  if (!gps_.testConnection())
  {
    throw runtime_error(ros::this_node::getName() + ": Failed to connect to GPS.");
  }

  if (!gps_.configureSolutionRate(SOLUTION_RATE))
  {
    ROS_ERROR_STREAM(ros::this_node::getName() << ": Failed to set solution rate.");
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

  double hor_std_dev = gps_data_[HOR_STD_DEV_IDX] / 1000;
  double ver_std_dev = gps_data_[VER_STD_DEV_IDX] / 1000;
  gps_msg_.position_covariance[0] = dh_std::sqr(hor_std_dev);
  gps_msg_.position_covariance[4] = dh_std::sqr(hor_std_dev);
  gps_msg_.position_covariance[8] = dh_std::sqr(ver_std_dev);

  // Publish GPS Message
  gps_pub_.publish(gps_msg_);
}
