#pragma once

#include <tobas_qt_tools/widgets/tab_widget.hpp>

#include "./plots/pose_plot.hpp"
#include "./plots/twist_plot.hpp"
#include "./plots/imu_plot.hpp"
#include "./plots/mag_plot.hpp"
#include "./plots/gnss_plot.hpp"
#include "./plots/battery_plot.hpp"
#include "./plots/rotor_speed_plot.hpp"
#include "./plots/latency_plot.hpp"
#include "./plots/dist_force_plot.hpp"
#include "./plots/observer_feedback_plot.hpp"

namespace gui
{
namespace log
{
class PlotTabWidget : public qt::TabWidget
{
  Q_OBJECT

  using self = PlotTabWidget;
  using super = qt::TabWidget;

  static constexpr int kTabWidth = 110;
  static constexpr int kTabHeight = 50;

public:
  explicit PlotTabWidget();

  void setTimeScale(double t_start, double t_stop);

  void setPoseData(const QVector<tobas_msgs::msg::Odometry>& _data);
  void setTwistData(const QVector<tobas_msgs::msg::Odometry>& _data);
  void setImuData(const QVector<tobas_msgs::msg::ImuWithCovarianceStamped>& _data);
  void setMagData(const QVector<tobas_msgs::msg::MagneticFieldWithCovarianceStamped>& _data);
  void setGnssData(const QVector<tobas_msgs::msg::Gnss>& _data);
  void setBatteryData(const QVector<tobas_msgs::msg::Battery>& _data);
  void setRotorSpeedData(
    const QVector<tobas_msgs::msg::RotorStateArray>& cur_data,
    const QVector<tobas_msgs::msg::RotorSpeedArray>& tar_data);
  void setSamplingTimeData(const QVector<tobas_msgs::msg::Latency>& _data);
  void setControlLatencyData(const QVector<tobas_msgs::msg::Latency>& _data);
  void setDisturbanceForceData(const QVector<tobas_kdl_msgs::msg::WrenchStamped>& _data);
  void setObserverFeedbackData(const QVector<tobas_debug_msgs::msg::ObserverFeedback>& _data);

private:
  PosePlotWidget* pose_plot_;
  TwistPlotWidget* twist_plot_;
  ImuPlotWidget* imu_plot_;
  MagPlotWidget* mag_plot_;
  GnssPlotWidget* gnss_plot_;
  BatteryPlotWidget* battery_plot_;
  RotorSpeedPlotWidget* rotor_speed_plot_;
  LatencyPlotWidget* latency_plot_;
  DisturbanceForcePlotWidget* dist_force_plot_;
  ObserverFeedbackPlotWidget* obsv_fb_plot_;
};
}  // namespace log
}  // namespace gui
