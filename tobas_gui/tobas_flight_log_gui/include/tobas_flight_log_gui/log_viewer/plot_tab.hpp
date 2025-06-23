#pragma once

#include <tobas_qt_tools/widgets/tab_widget.hpp>

#include "./plots/accel_plot.hpp"
#include "./plots/battery_plot.hpp"
#include "./plots/dist_force_plot.hpp"
#include "./plots/engine_plot.hpp"
#include "./plots/gnss_plot.hpp"
#include "./plots/imu_fft_plot.hpp"
#include "./plots/imu_plot.hpp"
#include "./plots/latency_plot.hpp"
#include "./plots/mag_plot.hpp"
#include "./plots/mr_controller_feedback_plot.hpp"
#include "./plots/observer_feedback_plot.hpp"
#include "./plots/pose_plot.hpp"
#include "./plots/propeller_pitch_plot.hpp"
#include "./plots/rotor_speed_plot.hpp"
#include "./plots/twist_plot.hpp"

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
  explicit PlotTabWidget(
    const QVector<tobas_msgs::msg::Odometry>& odom_data,
    const QVector<tobas_msgs::msg::ImuStamped>& raw_imu_data,
    const QVector<tobas_msgs::msg::ImuWithCovarianceStamped>& filt_imu_data,
    const QVector<tobas_msgs::msg::MagneticFieldWithCovarianceStamped>& mag_data,
    const QVector<tobas_msgs::msg::Gnss>& gnss_data,
    const QVector<tobas_msgs::msg::Battery>& battery_data,
    const QVector<tobas_msgs::msg::RotorStateArray>& cur_rotor_states_data,
    const QVector<tobas_msgs::msg::RotorSpeedArray>& tar_rotor_speeds_data,
    const QVector<tobas_msgs::msg::IcePropulsionSystemCommand>& ice_cmd_data,
    const QVector<tobas_msgs::msg::Latency>& sampling_time_data,
    const QVector<tobas_msgs::msg::Latency>& ctrl_latency_data,
    const QVector<tobas_kdl_msgs::msg::WrenchStamped>& dist_force_data,
    const QVector<tobas_debug_msgs::msg::ObserverFeedback>& obsv_fb_data,
    const QVector<tobas_debug_msgs::msg::MultiRotorControllerFeedback>& mr_ctrl_fb_data);

  void clear();
  void setTimeScale(double t_start, double t_stop);
  void plot();

private:
  const QVector<tobas_msgs::msg::Odometry>& odom_data_;
  const QVector<tobas_msgs::msg::ImuStamped>& raw_imu_data_;
  const QVector<tobas_msgs::msg::ImuWithCovarianceStamped>& filt_imu_data_;
  const QVector<tobas_msgs::msg::MagneticFieldWithCovarianceStamped>& mag_data_;
  const QVector<tobas_msgs::msg::Gnss>& gnss_data_;
  const QVector<tobas_msgs::msg::Battery>& battery_data_;
  const QVector<tobas_msgs::msg::RotorStateArray>& cur_rotor_states_data_;
  const QVector<tobas_msgs::msg::RotorSpeedArray>& tar_rotor_speeds_data_;
  const QVector<tobas_msgs::msg::IcePropulsionSystemCommand>& ice_cmd_data_;
  const QVector<tobas_msgs::msg::Latency>& sampling_time_data_;
  const QVector<tobas_msgs::msg::Latency>& ctrl_latency_data_;
  const QVector<tobas_kdl_msgs::msg::WrenchStamped>& dist_force_data_;
  const QVector<tobas_debug_msgs::msg::ObserverFeedback>& obsv_fb_data_;
  const QVector<tobas_debug_msgs::msg::MultiRotorControllerFeedback>& mr_ctrl_fb_data_;

  PosePlotWidget* pose_plot_;
  TwistPlotWidget* twist_plot_;
  AccelPlotWidget* accel_plot_;
  ImuPlotWidget* imu_plot_;
  ImuFftPlotWidget* imu_fft_plot_;
  MagPlotWidget* mag_plot_;
  GnssPlotWidget* gnss_plot_;
  BatteryPlotWidget* battery_plot_;
  EnginePlotWidget* engine_plot_;
  RotorSpeedPlotWidget* rotor_speed_plot_;
  PropellerPitchPlotWidget* propeller_pitch_plot_;
  LatencyPlotWidget* latency_plot_;
  DisturbanceForcePlotWidget* dist_force_plot_;
  ObserverFeedbackPlotWidget* obsv_fb_plot_;
  MRControllerFeedbackPlotWidget* mr_ctrl_fb_plot_;

  void plot(int index);

private Q_SLOTS:
  void onCurrentWidgetChanged(int index);
};
}  // namespace log
}  // namespace gui
