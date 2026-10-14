// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_flight_log_gui/log_viewer/plot_tab.hpp"

#include <tobas_qt_tools/cast.hpp>

namespace tobas::gui::log
{
PlotTabWidget::PlotTabWidget(
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
  : odom_data_(_odom_data)
  , setpoint_data_(_setpoint_data)
  , imu_raw_data_(_imu_raw_data)
  , imu_filt_data_(_imu_filt_data)
  , mag_data_(_mag_data)
  , pressure_data_(_pressure_data)
  , gnss_data_(_gnss_data)
  , rcin_data_(_rcin_data)
  , battery_data_(_battery_data)
  , cpu_data_(_cpu_data)
  , cur_rotor_states_data_(_cur_rotor_states_data)
  , tar_rotor_speeds_data_(_tar_rotor_speeds_data)
  , cur_joint_states_data_(_cur_joint_states_data)
  , tar_joint_positions_data_(_tar_joint_positions_data)
  , tar_joint_velocities_data_(_tar_joint_velocities_data)
  , tar_joint_efforts_data_(_tar_joint_efforts_data)
  , ice_cmd_data_(_ice_cmd_data)
  , pwm_data_(_pwm_data)
  , sampling_time_data_(_sampling_time_data)
  , ctrl_latency_data_(_ctrl_latency_data)
  , vibe_data_(_vibe_data)
  , additional_accel_data_(_additional_accel_data)
  , dist_force_data_(_dist_force_data)
  , obsv_fb_data_(_obsv_fb_data)
  , mr_ctrl_fb_data_(_mr_ctrl_fb_data)
{
  pose_plot_ = new PosePlotWidget();
  twist_plot_ = new TwistPlotWidget();
  accel_plot_ = new AccelPlotWidget();
  imu_plot_ = new ImuPlotWidget();
  imu_fft_plot_ = new ImuFftPlotWidget();
  mag_plot_ = new MagPlotWidget();
  pressure_plot_ = new AirPressurePlotWidget();
  gnss_plot_ = new GnssPlotWidget();
  rcin_plot_ = new RcInputPlotWidget();
  battery_plot_ = new BatteryPlotWidget();
  engine_plot_ = new EnginePlotWidget();
  cpu_plot_ = new CpuPlotWidget();
  rotor_speed_plot_ = new RotorSpeedPlotWidget();
  rotor_link_plot_ = new RotorLinkPlotWidget();
  propeller_pitch_plot_ = new PropellerPitchPlotWidget();
  joint_pos_plot_ = new JointPositionPlotWidget();
  joint_vel_plot_ = new JointVelocityPlotWidget();
  joint_eff_plot_ = new JointEffortPlotWidget();
  pwm_plot_ = new PwmPlotWidget();
  latency_plot_ = new LatencyPlotWidget();
  vibe_plot_ = new VibrationLevelPlotWidget();
  additional_accel_plot_ = new AdditionalAccelPlotWidget();
  dist_force_plot_ = new DisturbanceForcePlotWidget();
  obsv_fb_plot_ = new ObserverFeedbackPlotWidget();
  mr_ctrl_fb_plot_ = new MRControllerFeedbackPlotWidget();

  addTab(pose_plot_, "Pose");
  addTab(twist_plot_, "Twist");
  addTab(accel_plot_, "Accel");
  addTab(imu_plot_, "IMU");
  addTab(imu_fft_plot_, "IMU FFT");
  addTab(mag_plot_, "Magnetic\nField");
  addTab(pressure_plot_, "Air\nPressure");
  addTab(gnss_plot_, "GNSS");
  addTab(rcin_plot_, "RC Input");
  addTab(battery_plot_, "Battery");
  addTab(engine_plot_, "Engine");
  addTab(cpu_plot_, "CPU");
  addTab(rotor_speed_plot_, "Rotor Speed");
  addTab(rotor_link_plot_, "Rotor Link");
  addTab(propeller_pitch_plot_, "VPP Pitch");
  addTab(joint_pos_plot_, "Joint\nPosition");
  addTab(joint_vel_plot_, "Joint\nVelocity");
  addTab(joint_eff_plot_, "Joint\nEffort");
  addTab(pwm_plot_, "PWM");
  addTab(latency_plot_, "Latency");
  addTab(vibe_plot_, "Vibration\nLevel");
  addTab(additional_accel_plot_, "Additional\nAccel");
  addTab(dist_force_plot_, "Disturbance\nForce");
  addTab(obsv_fb_plot_, "Observer");
  addTab(mr_ctrl_fb_plot_, "Multirotor\nController");

  setTabSize(110, 50);

  connect(this, &self::currentChanged, this, &self::onCurrentWidgetChanged);
}

void PlotTabWidget::clear()
{
  for (int i = 0; i < count(); ++i) {
    const auto plot = qt::qPointerCast<BasePlotWidget>(widget(i));
    plot->clear();
  }
}

void PlotTabWidget::setTimeScale(double _t_start, double _t_stop)
{
  for (int i = 0; i < count(); ++i) {
    const auto plot = qt::qPointerCast<BasePlotWidget>(widget(i));
    plot->setTimeScale(_t_start, _t_stop);
  }
}

void PlotTabWidget::plot()
{
  plot(currentIndex());
}

void PlotTabWidget::plot(int _index)
{
  const auto cur_widget = widget(_index);

  if (cur_widget == pose_plot_) {
    pose_plot_->setData(odom_data_, setpoint_data_);
  }
  else if (cur_widget == twist_plot_) {
    twist_plot_->setData(odom_data_, setpoint_data_);
  }
  else if (cur_widget == accel_plot_) {
    accel_plot_->setData(odom_data_, setpoint_data_);
  }
  else if (cur_widget == imu_plot_) {
    imu_plot_->setData(imu_raw_data_, imu_filt_data_);
  }
  else if (cur_widget == imu_fft_plot_) {
    imu_fft_plot_->setData(imu_raw_data_, imu_filt_data_);
  }
  else if (cur_widget == mag_plot_) {
    mag_plot_->setData(mag_data_);
  }
  else if (cur_widget == pressure_plot_) {
    pressure_plot_->setData(pressure_data_);
  }
  else if (cur_widget == gnss_plot_) {
    gnss_plot_->setData(gnss_data_);
  }
  else if (cur_widget == rcin_plot_) {
    rcin_plot_->setData(rcin_data_);
  }
  else if (cur_widget == battery_plot_) {
    battery_plot_->setData(battery_data_);
  }
  else if (cur_widget == engine_plot_) {
    engine_plot_->setData(ice_cmd_data_);
  }
  else if (cur_widget == cpu_plot_) {
    cpu_plot_->setData(cpu_data_);
  }
  else if (cur_widget == rotor_speed_plot_) {
    rotor_speed_plot_->setData(cur_rotor_states_data_, tar_rotor_speeds_data_);
  }
  else if (cur_widget == rotor_link_plot_) {
    rotor_link_plot_->setData(cur_rotor_states_data_);
  }
  else if (cur_widget == propeller_pitch_plot_) {
    propeller_pitch_plot_->setData(ice_cmd_data_);
  }
  else if (cur_widget == joint_pos_plot_) {
    joint_pos_plot_->setData(cur_joint_states_data_, tar_joint_positions_data_);
  }
  else if (cur_widget == joint_vel_plot_) {
    joint_vel_plot_->setData(cur_joint_states_data_, tar_joint_velocities_data_);
  }
  else if (cur_widget == joint_eff_plot_) {
    joint_eff_plot_->setData(cur_joint_states_data_, tar_joint_efforts_data_);
  }
  else if (cur_widget == pwm_plot_) {
    pwm_plot_->setData(pwm_data_);
  }
  else if (cur_widget == latency_plot_) {
    latency_plot_->setData(sampling_time_data_, ctrl_latency_data_);
  }
  else if (cur_widget == vibe_plot_) {
    vibe_plot_->setData(vibe_data_);
  }
  else if (cur_widget == additional_accel_plot_) {
    additional_accel_plot_->setData(additional_accel_data_);
  }
  else if (cur_widget == dist_force_plot_) {
    dist_force_plot_->setData(dist_force_data_);
  }
  else if (cur_widget == obsv_fb_plot_) {
    obsv_fb_plot_->setData(obsv_fb_data_);
  }
  else if (cur_widget == mr_ctrl_fb_plot_) {
    mr_ctrl_fb_plot_->setData(mr_ctrl_fb_data_);
  }
  else {
    qWarning() << "Invalid index:" << _index;
  }
}

void PlotTabWidget::onCurrentWidgetChanged(int _index)
{
  plot(_index);
}
}  // namespace tobas::gui::log
