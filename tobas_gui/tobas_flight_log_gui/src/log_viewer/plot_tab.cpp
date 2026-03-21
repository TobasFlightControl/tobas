#include "tobas_flight_log_gui/log_viewer/plot_tab.hpp"

#include <tobas_qt_tools/cast.hpp>

namespace gui
{
namespace log
{
PlotTabWidget::PlotTabWidget(
  const QVector<tobas_msgs::msg::OdometryWithCovarianceStamped>& odom_data,
  const QVector<tobas_msgs::msg::OdometryStamped>& setpoint_data,
  const QVector<tobas_msgs::msg::Imu>& raw_imu_data,
  const QVector<tobas_msgs::msg::Imu>& filt_imu_data,
  const QVector<tobas_msgs::msg::MagneticField>& mag_data,
  const QVector<tobas_msgs::msg::Gnss>& gnss_data,
  const QVector<tobas_msgs::msg::RCInput>& rcin_data,
  const QVector<tobas_msgs::msg::Battery>& battery_data,
  const QVector<tobas_msgs::msg::Cpu>& cpu_data,
  const QVector<tobas_msgs::msg::RotorStateArray>& cur_rotor_states_data,
  const QVector<tobas_msgs::msg::RotorSpeedArray>& tar_rotor_speeds_data,
  const QVector<tobas_msgs::msg::JointStateArray>& cur_joint_states_data,
  const QVector<tobas_msgs::msg::JointCommandArray>& tar_joint_positions_data,
  const QVector<tobas_msgs::msg::JointCommandArray>& tar_joint_velocities_data,
  const QVector<tobas_msgs::msg::JointCommandArray>& tar_joint_efforts_data,
  const QVector<tobas_msgs::msg::IcePropulsionSystemCommand>& ice_cmd_data,
  const QVector<tobas_msgs::msg::Latency>& sampling_time_data,
  const QVector<tobas_msgs::msg::Latency>& ctrl_latency_data,
  const QVector<tobas_msgs::msg::VibrationLevel>& vibe_data,
  const QVector<tobas_kdl_msgs::msg::WrenchStamped>& dist_force_data,
  const QVector<tobas_debug_msgs::msg::ObserverFeedback>& obsv_fb_data,
  const QVector<tobas_debug_msgs::msg::MulticopterControllerFeedback>& mr_ctrl_fb_data)
  : odom_data_(odom_data)
  , setpoint_data_(setpoint_data)
  , raw_imu_data_(raw_imu_data)
  , filt_imu_data_(filt_imu_data)
  , mag_data_(mag_data)
  , gnss_data_(gnss_data)
  , rcin_data_(rcin_data)
  , battery_data_(battery_data)
  , cpu_data_(cpu_data)
  , cur_rotor_states_data_(cur_rotor_states_data)
  , tar_rotor_speeds_data_(tar_rotor_speeds_data)
  , cur_joint_states_data_(cur_joint_states_data)
  , tar_joint_positions_data_(tar_joint_positions_data)
  , tar_joint_velocities_data_(tar_joint_velocities_data)
  , tar_joint_efforts_data_(tar_joint_efforts_data)
  , ice_cmd_data_(ice_cmd_data)
  , sampling_time_data_(sampling_time_data)
  , ctrl_latency_data_(ctrl_latency_data)
  , vibe_data_(vibe_data)
  , dist_force_data_(dist_force_data)
  , obsv_fb_data_(obsv_fb_data)
  , mr_ctrl_fb_data_(mr_ctrl_fb_data)
{
  pose_plot_ = new PosePlotWidget();
  twist_plot_ = new TwistPlotWidget();
  accel_plot_ = new AccelPlotWidget();
  imu_plot_ = new ImuPlotWidget();
  imu_fft_plot_ = new ImuFftPlotWidget();
  mag_plot_ = new MagPlotWidget();
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
  latency_plot_ = new LatencyPlotWidget();
  vibe_plot_ = new VibrationLevelPlotWidget();
  dist_force_plot_ = new DisturbanceForcePlotWidget();
  obsv_fb_plot_ = new ObserverFeedbackPlotWidget();
  mr_ctrl_fb_plot_ = new MRControllerFeedbackPlotWidget();

  addTab(pose_plot_, "Pose");
  addTab(twist_plot_, "Twist");
  addTab(accel_plot_, "Accel");
  addTab(imu_plot_, "IMU");
  addTab(imu_fft_plot_, "IMU FFT");
  addTab(mag_plot_, "Magnetic\nField");
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
  addTab(latency_plot_, "Latency");
  addTab(vibe_plot_, "Vibration\nLevel");
  addTab(dist_force_plot_, "Disturbance\nForce");
  addTab(obsv_fb_plot_, "Observer");
  addTab(mr_ctrl_fb_plot_, "Multirotor\nController");

  setTabSize(kTabWidth, kTabHeight);

  connect(this, &self::currentChanged, this, &self::onCurrentWidgetChanged);
}

void PlotTabWidget::clear()
{
  for (int i = 0; i < count(); ++i) {
    const auto plot = qt::qPointerCast<BasePlotWidget>(widget(i));
    plot->clear();
  }
}

void PlotTabWidget::setTimeScale(double t_start, double t_stop)
{
  for (int i = 0; i < count(); ++i) {
    const auto plot = qt::qPointerCast<BasePlotWidget>(widget(i));
    plot->setTimeScale(t_start, t_stop);
  }
}

void PlotTabWidget::plot()
{
  plot(currentIndex());
}

void PlotTabWidget::plot(int index)
{
  const auto cur_widget = widget(index);

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
    imu_plot_->setData(raw_imu_data_, filt_imu_data_);
  }
  else if (cur_widget == imu_fft_plot_) {
    imu_fft_plot_->setData(raw_imu_data_);
  }
  else if (cur_widget == mag_plot_) {
    mag_plot_->setData(mag_data_);
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
  else if (cur_widget == latency_plot_) {
    latency_plot_->setData(sampling_time_data_, ctrl_latency_data_);
  }
  else if (cur_widget == vibe_plot_) {
    vibe_plot_->setData(vibe_data_);
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
    qWarning() << "Invalid index:" << index;
  }
}

void PlotTabWidget::onCurrentWidgetChanged(int index)
{
  plot(index);
}
}  // namespace log
}  // namespace gui
