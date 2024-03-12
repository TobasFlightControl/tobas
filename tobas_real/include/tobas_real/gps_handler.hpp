#pragma once

#include <ros/ros.h>
#include <ros/timer.h>

#include <navio2/ublox.hpp>
#include <tobas_tools/node.hpp>

namespace tobas_real
{
class GpsHandler : public tobas::BaseNode
{
  // GPSレシーバの更新周期 [ms]
  // 周波数が高すぎるとFIFOにデータが溜まってタイムシフトが生じるため，そんなに大きくできない
  static constexpr size_t kMeasurementRate = 1000 / 5;

  using self = GpsHandler;
  using super = tobas::BaseNode;

public:
  explicit GpsHandler(
    const ros::NodeHandle& nh,
    const ros::NodeHandle& pnh,
    const std::string& name = ros::this_node::getName());

private:
  navio::Ublox gps_;
  navio::NavPvtPayload pvt_;
  navio::NavCovPayload cov_;
  navio::NavTimeutcPayload timeutc_;
  bool cov_received_ = false;

  // Publisher
  ros::Publisher gps_pub_;

  // Timer
  ros::Timer main_timer_;

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;

  void configureGnssReceiver();

  void mainTimerCb(const ros::TimerEvent& event);
};
}  // namespace tobas_real
