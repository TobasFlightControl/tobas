// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_flight_log_gui/log_viewer/plot_panel.hpp"

#include <QComboBox>
#include <QHBoxLayout>
#include <QLayout>
#include <QStyle>
#include <QVBoxLayout>

#include "tobas_flight_log_gui/log_viewer/plots/accel_plot.hpp"
#include "tobas_flight_log_gui/log_viewer/plots/additional_accel_plot.hpp"
#include "tobas_flight_log_gui/log_viewer/plots/air_pressure_plot.hpp"
#include "tobas_flight_log_gui/log_viewer/plots/battery_plot.hpp"
#include "tobas_flight_log_gui/log_viewer/plots/cpu_plot.hpp"
#include "tobas_flight_log_gui/log_viewer/plots/dist_force_plot.hpp"
#include "tobas_flight_log_gui/log_viewer/plots/engine_plot.hpp"
#include "tobas_flight_log_gui/log_viewer/plots/gnss_plot.hpp"
#include "tobas_flight_log_gui/log_viewer/plots/imu_fft_plot.hpp"
#include "tobas_flight_log_gui/log_viewer/plots/imu_plot.hpp"
#include "tobas_flight_log_gui/log_viewer/plots/joint_effort_plot.hpp"
#include "tobas_flight_log_gui/log_viewer/plots/joint_position_plot.hpp"
#include "tobas_flight_log_gui/log_viewer/plots/joint_velocity_plot.hpp"
#include "tobas_flight_log_gui/log_viewer/plots/latency_plot.hpp"
#include "tobas_flight_log_gui/log_viewer/plots/mag_plot.hpp"
#include "tobas_flight_log_gui/log_viewer/plots/mr_controller_feedback_plot.hpp"
#include "tobas_flight_log_gui/log_viewer/plots/observer_feedback_plot.hpp"
#include "tobas_flight_log_gui/log_viewer/plots/pose_plot.hpp"
#include "tobas_flight_log_gui/log_viewer/plots/propeller_pitch_plot.hpp"
#include "tobas_flight_log_gui/log_viewer/plots/pwm_plot.hpp"
#include "tobas_flight_log_gui/log_viewer/plots/rc_input_plot.hpp"
#include "tobas_flight_log_gui/log_viewer/plots/rotor_link_plot.hpp"
#include "tobas_flight_log_gui/log_viewer/plots/rotor_speed_plot.hpp"
#include "tobas_flight_log_gui/log_viewer/plots/twist_plot.hpp"
#include "tobas_flight_log_gui/log_viewer/plots/utilities/base_plot.hpp"
#include "tobas_flight_log_gui/log_viewer/plots/vibration_level_plot.hpp"

namespace tobas::gui::log
{
PlotPanelWidget::PlotPanelWidget(
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
  const QVector<tobas_debug_msgs::msg::MulticopterControllerFeedback>& _mr_ctrl_fb_data)
{
  plot_selector_ = new qt::ComboBox();
  plot_stack_ = new QStackedWidget();

  expand_button_ = new QToolButton();
  expand_button_->setCheckable(true);
  expand_button_->setIcon(style()->standardIcon(QStyle::SP_TitleBarMaxButton));
  expand_button_->setToolTip("Expand plot");

  addPlot<PosePlotWidget>("Pose", _odom_data, _setpoint_data);
  addPlot<TwistPlotWidget>("Twist", _odom_data, _setpoint_data);
  addPlot<AccelPlotWidget>("Accel", _odom_data, _setpoint_data);
  addPlot<ImuPlotWidget>("IMU", _imu_raw_data, _imu_filt_data);
  addPlot<ImuFftPlotWidget>("IMU FFT", _imu_raw_data, _imu_filt_data);
  addPlot<MagPlotWidget>("Magnetic Field", _mag_data);
  addPlot<AirPressurePlotWidget>("Air Pressure", _pressure_data);
  addPlot<GnssPlotWidget>("GNSS", _gnss_data);
  addPlot<RcInputPlotWidget>("RC Input", _rcin_data);
  addPlot<BatteryPlotWidget>("Battery", _battery_data);
  addPlot<EnginePlotWidget>("Engine", _ice_cmd_data);
  addPlot<CpuPlotWidget>("CPU", _cpu_data);
  addPlot<RotorSpeedPlotWidget>("Rotor Speed", _cur_rotor_states_data, _tar_rotor_speeds_data);
  addPlot<RotorLinkPlotWidget>("Rotor Link", _cur_rotor_states_data);
  addPlot<PropellerPitchPlotWidget>("VPP Pitch", _ice_cmd_data);
  addPlot<JointPositionPlotWidget>("Joint Position", _cur_joint_states_data, _tar_joint_positions_data);
  addPlot<JointVelocityPlotWidget>("Joint Velocity", _cur_joint_states_data, _tar_joint_velocities_data);
  addPlot<JointEffortPlotWidget>("Joint Effort", _cur_joint_states_data, _tar_joint_efforts_data);
  addPlot<PwmPlotWidget>("PWM", _pwm_data);
  addPlot<LatencyPlotWidget>("Latency", _sampling_time_data, _ctrl_latency_data);
  addPlot<VibrationLevelPlotWidget>("Vibration Level", _vibe_data);
  addPlot<AdditionalAccelPlotWidget>("Additional Accel", _additional_accel_data);
  addPlot<DisturbanceForcePlotWidget>("Disturbance Force", _dist_force_data);
  addPlot<ObserverFeedbackPlotWidget>("Observer", _obsv_fb_data);
  addPlot<MRControllerFeedbackPlotWidget>("Multirotor Controller", _mr_ctrl_fb_data);

  const auto header = new QHBoxLayout();
  header->addWidget(plot_selector_, 0);
  header->addStretch(1);
  header->addWidget(expand_button_, 0);

  const auto rows = new QVBoxLayout(this);
  rows->setContentsMargins(0, 0, 0, 0);
  rows->setSpacing(4);
  rows->addLayout(header, 0);
  rows->addWidget(plot_stack_, 1);

  connect(plot_selector_, &QComboBox::currentIndexChanged, this, &self::onCurrentWidgetChanged);
  connect(expand_button_, &QToolButton::toggled, this, &self::onExpandButtonToggled);
}

void PlotPanelWidget::setCurrentIndex(int _index)
{
  if (_index < 0 || _index >= static_cast<int>(plots_.size())) {
    qWarning() << "Invalid index:" << _index;
    return;
  }
  plot_selector_->setCurrentIndex(_index);
  currentPlot();
}

void PlotPanelWidget::clear()
{
  for (const auto& entry : plots_) {
    if (entry.plot.widget) {
      entry.plot.widget->clear();
    }
  }
}

void PlotPanelWidget::setTimeScale(double _t_start, double _t_stop)
{
  t_start_ = _t_start;
  t_stop_ = _t_stop;

  currentPlot().widget->setTimeScale(_t_start, _t_stop);
}

void PlotPanelWidget::plot()
{
  currentPlot().update();
}

template <typename PlotType, typename... MsgTypes>
void PlotPanelWidget::addPlot(const QString& _name, const QVector<MsgTypes>&... _data)
{
  // The viewer owns the data vectors and keeps them alive for the lifetime of its panels.
  const auto create = [&_data...]
  {
    const auto widget = new PlotType();
    widget->layout()->setContentsMargins(0, 0, 0, 0);
    widget->layout()->setSpacing(0);
    widget->clear();
    return Plot{ widget, [widget, &_data...] { widget->setData(_data...); } };
  };
  plots_.push_back({ {}, create });

  plot_selector_->addItem(_name);
}

PlotPanelWidget::Plot& PlotPanelWidget::currentPlot()
{
  const auto index = plot_selector_->currentIndex();
  auto& entry = plots_[index];
  if (!entry.plot.widget) {
    entry.plot = entry.create();
    entry.create = {};
    plot_stack_->addWidget(entry.plot.widget);
  }
  return entry.plot;
}

void PlotPanelWidget::onCurrentWidgetChanged()
{
  auto& plot = currentPlot();
  plot_stack_->setCurrentWidget(plot.widget);
  plot.widget->setTimeScale(t_start_, t_stop_);
  plot.update();
}

void PlotPanelWidget::onExpandButtonToggled(bool _expanded)
{
  expand_button_->setIcon(
    style()->standardIcon(_expanded ? QStyle::SP_TitleBarNormalButton : QStyle::SP_TitleBarMaxButton));
  const auto label = _expanded ? "Restore layout" : "Expand plot";
  expand_button_->setToolTip(label);

  Q_EMIT expandedChanged(_expanded);
}
}  // namespace tobas::gui::log
