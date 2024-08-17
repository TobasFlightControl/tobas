#pragma once

#include <tobas_hal_core/base_sensor_node.hpp>
#include <tobas_a1_core/zed_f9p.hpp>

namespace a1
{
class GNSSDriver : public hal::BaseSensorNode
{
  // GNSSレシーバの更新周期 [ms]
  // 周波数が高すぎるとFIFOにデータが溜まってタイムシフトが生じるため，そんなに大きくできない
  static constexpr size_t kMeasPeriod = 1000 / 20;

  using self = GNSSDriver;
  using super = hal::BaseSensorNode;

public:
  explicit GNSSDriver(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  ZEDF9P gnss_;

  payload::NAV_STATUS status_;
  payload::NAV_HPPOSLLH hpposllh_;
  payload::NAV_VELNED velned_;
  payload::NAV_COV cov_;

  std::map<ZEDF9P::ubx_nav_id_t, bool> is_received_;
  rclcpp::Duration time_offset_;  // ROS Time - GPS Time

  PublisherPtr<> gnss_pub_;
  rclcpp::Timer set_time_offset_timer_;

  bool configure();
  void warnUnnecessaryUBXMessage();

  void setTimeOffsetTimerCb();
  void mainTimerCb();
};
}  // namespace a1
