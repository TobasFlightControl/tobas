#pragma once

#include <ros/ros.h>
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
  static constexpr uint32_t kSleepTime = 200;  // [us]

  using super = tobas::BaseNode;

  using GpsMsg = sensor_msgs::NavSatFix;
  using VelMsg = tobas_msgs::LinearVelocityWithCovariance;

public:
  explicit GpsHandler();

  void run();

private:
  Ublox gps_;
  NavStatusPayload status_;
  NavPvtPayload pvt_;
  NavCovPayload cov_;
  GpsMsg gps_msg_;
  VelMsg vel_msg_;
  bool gps_fix_ok_;
  bool cov_received_;

  // Publisher
  ros::Publisher gps_pub_;
  ros::Publisher vel_pub_;

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;

  void configureGnssReceiver();
  bool isReadyToPublish() const;

  void eventCb(const tobas_msgs::Event& event) override;
};
}  // namespace tobas_real
