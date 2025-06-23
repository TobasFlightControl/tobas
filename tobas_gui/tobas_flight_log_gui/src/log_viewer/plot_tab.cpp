#include "tobas_flight_log_gui/log_viewer/plot_tab.hpp"

namespace gui
{
namespace log
{
PlotTabWidget::PlotTabWidget()
{
  pose_plot_ = new PosePlotWidget();
  twist_plot_ = new TwistPlotWidget();
  accel_plot_ = new AccelPlotWidget();
  imu_plot_ = new ImuPlotWidget();
  imu_fft_plot_ = new ImuFftPlotWidget();
  mag_plot_ = new MagPlotWidget();
  gnss_plot_ = new GnssPlotWidget();
  battery_plot_ = new BatteryPlotWidget();
  engine_plot_ = new EnginePlotWidget();
  rotor_speed_plot_ = new RotorSpeedPlotWidget();
  propeller_pitch_plot_ = new PropellerPitchPlotWidget();
  latency_plot_ = new LatencyPlotWidget();
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
  addTab(battery_plot_, "Battery");
  addTab(engine_plot_, "Engine");
  addTab(rotor_speed_plot_, "Rotor Speed");
  addTab(propeller_pitch_plot_, "VPP Pitch");
  addTab(latency_plot_, "Latency");
  addTab(dist_force_plot_, "Disturbance\nForce");
  addTab(obsv_fb_plot_, "Observer");
  addTab(mr_ctrl_fb_plot_, "Multirotor\nController");

  setTabSize(kTabWidth, kTabHeight);
}

void PlotTabWidget::clear()
{
  rotor_speed_plot_->clear();
  propeller_pitch_plot_->clear();
}

void PlotTabWidget::setTimeScale(double t_start, double t_stop)
{
  pose_plot_->setTimeScale(t_start, t_stop);
  twist_plot_->setTimeScale(t_start, t_stop);
  accel_plot_->setTimeScale(t_start, t_stop);
  imu_plot_->setTimeScale(t_start, t_stop);
  mag_plot_->setTimeScale(t_start, t_stop);
  gnss_plot_->setTimeScale(t_start, t_stop);
  battery_plot_->setTimeScale(t_start, t_stop);
  engine_plot_->setTimeScale(t_start, t_stop);
  rotor_speed_plot_->setTimeScale(t_start, t_stop);
  propeller_pitch_plot_->setTimeScale(t_start, t_stop);
  latency_plot_->setTimeScale(t_start, t_stop);
  dist_force_plot_->setTimeScale(t_start, t_stop);
  obsv_fb_plot_->setTimeScale(t_start, t_stop);
  mr_ctrl_fb_plot_->setTimeScale(t_start, t_stop);
}

void PlotTabWidget::setData(
  const QVector<tobas_msgs::msg::Odometry> odom_data,
  const QVector<tobas_msgs::msg::ImuStamped> raw_imu_data,
  const QVector<tobas_msgs::msg::ImuWithCovarianceStamped> filt_imu_data,
  const QVector<tobas_msgs::msg::MagneticFieldWithCovarianceStamped> mag_data,
  const QVector<tobas_msgs::msg::Gnss> gnss_data,
  const QVector<tobas_msgs::msg::Battery> battery_data,
  const QVector<tobas_msgs::msg::RotorStateArray> cur_rotor_states_data,
  const QVector<tobas_msgs::msg::RotorSpeedArray> tar_rotor_speeds_data,
  const QVector<tobas_msgs::msg::IcePropulsionSystemCommand> ice_cmd_data,
  const QVector<tobas_msgs::msg::Latency> sampling_time_data,
  const QVector<tobas_msgs::msg::Latency> ctrl_latency_data,
  const QVector<tobas_kdl_msgs::msg::WrenchStamped> dist_force_data,
  const QVector<tobas_debug_msgs::msg::ObserverFeedback> obsv_fb_data,
  const QVector<tobas_debug_msgs::msg::MultiRotorControllerFeedback> mr_ctrl_fb_data)
{
  pose_plot_->setData(odom_data, mr_ctrl_fb_data);
  twist_plot_->setData(odom_data, mr_ctrl_fb_data);
  accel_plot_->setData(odom_data, mr_ctrl_fb_data);
  imu_plot_->setData(raw_imu_data, filt_imu_data);
  imu_fft_plot_->setData(raw_imu_data);
  mag_plot_->setData(mag_data);
  gnss_plot_->setData(gnss_data);
  battery_plot_->setData(battery_data);
  engine_plot_->setData(ice_cmd_data);
  rotor_speed_plot_->setData(cur_rotor_states_data, tar_rotor_speeds_data);
  propeller_pitch_plot_->setData(ice_cmd_data);
  latency_plot_->setData(sampling_time_data, ctrl_latency_data);
  dist_force_plot_->setData(dist_force_data);
  obsv_fb_plot_->setData(obsv_fb_data);
  mr_ctrl_fb_plot_->setData(mr_ctrl_fb_data);
}
}  // namespace log
}  // namespace gui
