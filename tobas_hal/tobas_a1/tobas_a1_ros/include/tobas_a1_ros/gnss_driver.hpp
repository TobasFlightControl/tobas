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
  explicit GNSSDriver(ros::NodeHandle& nh, ros::NodeHandle& pnh, const std::string& name = ros::this_node::getName());

private:
  ZEDF9P gnss_;

  payload::NAV_STATUS status_;
  payload::NAV_HPPOSLLH hpposllh_;
  payload::NAV_VELNED velned_;
  payload::NAV_COV cov_;

  std::map<ZEDF9P::ubx_nav_id_t, bool> is_received_;
  ros::Duration time_offset_;  // ROS Time - GPS Time

  ros::Publisher gnss_pub_;
  ros::Timer set_time_offset_timer_;

  bool configure();
  void warnUnnecessaryUBXMessage();

  void setTimeOffsetTimerCb(const ros::TimerEvent& event);
  void mainTimerCb(const ros::TimerEvent& event);
};
}  // namespace a1
