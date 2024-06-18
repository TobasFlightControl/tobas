#pragma once

#include <tobas_navio_core/ublox.hpp>

#include "./base_sensor_node.hpp"

namespace tobas_navio_ros
{
class GpsHandler : public BaseSensorNode
{
  // GPSレシーバの更新周期 [ms]
  // 周波数が高すぎるとFIFOにデータが溜まってタイムシフトが生じるため，そんなに大きくできない
  static constexpr size_t kMeasurementRate = 1000 / 5;

  using self = GpsHandler;
  using super = BaseSensorNode;

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

  void configureGnssReceiver();
  void mainTimerCb(const ros::TimerEvent& event);
};
}  // namespace tobas_navio_ros
