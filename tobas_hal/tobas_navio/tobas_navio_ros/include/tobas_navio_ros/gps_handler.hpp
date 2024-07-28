#pragma once

#include <tobas_hal_core/base_sensor_node.hpp>
#include <tobas_navio_core/neo_m8n.hpp>

namespace tobas_navio_ros
{
class GpsHandler : public hal::BaseSensorNode
{
  // GPSレシーバの更新周期 [ms]
  // 周波数が高すぎるとFIFOにデータが溜まってタイムシフトが生じるため，そんなに大きくできない
  static constexpr size_t kMeasurementRate = 1000 / 5;

  using self = GpsHandler;
  using super = hal::BaseSensorNode;

public:
  explicit GpsHandler(ros::NodeHandle& nh, ros::NodeHandle& pnh, const std::string& name = ros::this_node::getName());

private:
  navio::NEOM8N gps_;
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
