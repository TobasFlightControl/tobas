#pragma once

#include <ros/ros.h>
#include <ros/timer.h>
#include <sensor_msgs/NavSatFix.h>

#include <Common/Ublox.h>

#include <tobas_tools/node.hpp>
#include <tobas_msgs/LinearVelocityWithCovariance.h>

namespace tobas_real
{
class GpsHandler : public tobas::BaseNode
{
  // GPSレシーバの更新周期[ms]．10Hzにするとレシーバの処理が間に合わず遅延が発生する．
  static constexpr uint32_t kMeasurementRate = 200;
  static constexpr double kSleepTime = 1e-3;  // [s]

  using super = tobas::BaseNode;

  using GpsMsg = sensor_msgs::NavSatFix;
  using VelMsg = tobas_msgs::LinearVelocityWithCovariance;

public:
  explicit GpsHandler(
    ros::NodeHandle nh,
    ros::NodeHandle pnh,
    std::string name = ros::this_node::getName());

private:
  Ublox gps_;
  NavPvtPayload pvt_;
  NavCovPayload cov_;
  NavTimeutcPayload timeutc_;
  bool cov_received_ = false;

  // Publisher
  ros::Publisher gps_pub_;
  ros::Publisher vel_pub_;

  // Timer
  ros::Timer main_timer_;

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;

  void configureGnssReceiver();

  void eventCb(const tobas_msgs::EventConstPtr& event) override;
  void mainTimerCb(const ros::TimerEvent& event);
};
}  // namespace tobas_real
