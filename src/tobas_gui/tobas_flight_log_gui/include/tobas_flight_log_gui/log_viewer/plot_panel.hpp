// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <functional>
#include <vector>

#include <QStackedWidget>
#include <QString>
#include <QToolButton>
#include <QVector>
#include <QWidget>

#include <tobas_qt_tools/widgets/combo_box.hpp>

#include <tobas_debug_msgs/msg/multicopter_controller_feedback.hpp>
#include <tobas_debug_msgs/msg/observer_feedback.hpp>
#include <tobas_kdl_msgs/msg/accel_stamped.hpp>
#include <tobas_kdl_msgs/msg/wrench_stamped.hpp>
#include <tobas_msgs/msg/battery.hpp>
#include <tobas_msgs/msg/cpu.hpp>
#include <tobas_msgs/msg/fluid_pressure.hpp>
#include <tobas_msgs/msg/gnss.hpp>
#include <tobas_msgs/msg/ice_propulsion_system_command.hpp>
#include <tobas_msgs/msg/imu.hpp>
#include <tobas_msgs/msg/joint_command_array.hpp>
#include <tobas_msgs/msg/joint_state_array.hpp>
#include <tobas_msgs/msg/latency.hpp>
#include <tobas_msgs/msg/magnetic_field.hpp>
#include <tobas_msgs/msg/odometry_stamped.hpp>
#include <tobas_msgs/msg/odometry_with_covariance_stamped.hpp>
#include <tobas_msgs/msg/pwm_array.hpp>
#include <tobas_msgs/msg/rc_input.hpp>
#include <tobas_msgs/msg/rotor_speed_array.hpp>
#include <tobas_msgs/msg/rotor_state_array.hpp>
#include <tobas_msgs/msg/vibration_level.hpp>

#include "./plots/utilities/base_plot.hpp"

namespace tobas::gui::log
{
class PlotPanelWidget : public QWidget
{
  Q_OBJECT

  using self = PlotPanelWidget;

  struct Plot
  {
    BasePlotWidget* widget = nullptr;
    std::function<void()> update;
  };

  struct PlotEntry
  {
    Plot plot;
    std::function<Plot()> create;
  };

Q_SIGNALS:
  /** Request expansion when `_expanded` is true, or restoration when false. */
  void expandedChanged(bool _expanded);

public:
  /**
   * Create the panel controls and register plots for lazy creation using the supplied data vectors.
   *
   * @note The data vectors are referenced and must remain valid while the panel is in use.
   */
  explicit PlotPanelWidget(
    const QVector<tobas_msgs::msg::OdometryWithCovarianceStamped>& _odom_data,
    const QVector<tobas_msgs::msg::OdometryStamped>& _setpoint_data,
    const QVector<tobas_msgs::msg::Imu>& _imu_raw_data,
    const QVector<tobas_msgs::msg::Imu>& _imu_filt_data,
    const QVector<tobas_msgs::msg::MagneticField>& _mag_data,
    const QVector<tobas_msgs::msg::FluidPressure>& _pressure_data,
    const QVector<tobas_msgs::msg::Gnss>& _gnss_data,
    const QVector<tobas_msgs::msg::RCInput>& _rcin_data,
    const QVector<tobas_msgs::msg::Battery>& _battery_data,
    const QVector<tobas_msgs::msg::Cpu>& _cpu_data,
    const QVector<tobas_msgs::msg::RotorStateArray>& _cur_rotor_states_data,
    const QVector<tobas_msgs::msg::RotorSpeedArray>& _tar_rotor_speeds_data,
    const QVector<tobas_msgs::msg::JointStateArray>& _cur_joint_states_data,
    const QVector<tobas_msgs::msg::JointCommandArray>& _tar_joint_positions_data,
    const QVector<tobas_msgs::msg::JointCommandArray>& _tar_joint_velocities_data,
    const QVector<tobas_msgs::msg::JointCommandArray>& _tar_joint_efforts_data,
    const QVector<tobas_msgs::msg::IcePropulsionSystemCommand>& _ice_cmd_data,
    const QVector<tobas_msgs::msg::PwmArray>& _pwm_data,
    const QVector<tobas_msgs::msg::Latency>& _sampling_time_data,
    const QVector<tobas_msgs::msg::Latency>& _ctrl_latency_data,
    const QVector<tobas_msgs::msg::VibrationLevel>& _vibe_data,
    const QVector<tobas_kdl_msgs::msg::AccelStamped>& _additional_accel_data,
    const QVector<tobas_kdl_msgs::msg::WrenchStamped>& _dist_force_data,
    const QVector<tobas_debug_msgs::msg::ObserverFeedback>& _obsv_fb_data,
    const QVector<tobas_debug_msgs::msg::MulticopterControllerFeedback>& _mr_ctrl_fb_data);

  /** Select a plot by registration index and create it on first use, warning on invalid indices. */
  void setCurrentIndex(int _index);

  /** Clear all created plots and retain their widgets for reuse. */
  void clear();

  /** Store the time range in seconds and pass it to the selected plot, creating the plot if needed. */
  void setTimeScale(double _t_start, double _t_stop);

  /** Create the selected plot if needed and refresh it from the referenced data vectors. */
  void plot();

private:
  qt::ComboBox* plot_selector_;
  QStackedWidget* plot_stack_;
  QToolButton* expand_button_;

  std::vector<PlotEntry> plots_;

  double t_start_ = 0.0;
  double t_stop_ = 1.0;

  /** Register a named plot with data references and defer widget creation until first use. */
  template <typename PlotType, typename... MsgTypes>
  void addPlot(const QString& _name, const QVector<MsgTypes>&... _data);

  /** Return the selected plot, creating it and adding its widget to the stack on first use. */
  Plot& currentPlot();

private Q_SLOTS:
  /** Show the selected plot, apply the stored time range, and refresh its data. */
  void onCurrentWidgetChanged();

  /** Update the expand button and emit `expandedChanged()` for the requested layout state. */
  void onExpandButtonToggled(bool _expanded);
};
}  // namespace tobas::gui::log
