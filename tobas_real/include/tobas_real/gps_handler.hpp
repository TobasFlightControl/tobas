#pragma once

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>

#include <Common/Ublox.h>

#include <dh_ros_tools/node.hpp>

#include <tobas_msgs/LinearVelocityWithCovariance.h>

class GpsHandler : public dh_ros::BaseNode
{
  using super = dh_ros::BaseNode;

  using GpsMsg = sensor_msgs::NavSatFix;
  using VelMsg = tobas_msgs::LinearVelocityWithCovariance;

public:
  explicit GpsHandler();

private:
  Ublox gps_;
  std::vector<double> data_;
  GpsMsg gps_msg_;
  VelMsg vel_msg_;
  bool cov_received_;

  // Publisher
  ros::Publisher gps_pub_;
  ros::Publisher vel_pub_;

  ros::Timer main_loop_timer_;

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;
  void createTimers() override;

  void checkTopicsTimerCb(const ros::TimerEvent& event) override;
  void mainLoopTimerCb(const ros::TimerEvent&);
};
