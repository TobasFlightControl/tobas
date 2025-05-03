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

void PlotTabWidget::setFrameData(
  const QVector<tobas_msgs::msg::Odometry>& odom_data,
  const QVector<tobas_debug_msgs::msg::MultiRotorControllerFeedback>& ctrl_fb_data)
{
  pose_plot_->setData(odom_data, ctrl_fb_data);
  twist_plot_->setData(odom_data, ctrl_fb_data);
  accel_plot_->setData(odom_data, ctrl_fb_data);
}

void PlotTabWidget::setImuData(const QVector<tobas_msgs::msg::ImuWithCovarianceStamped>& _data)
{
  imu_plot_->setData(_data);
}

void PlotTabWidget::setMagData(const QVector<tobas_msgs::msg::MagneticFieldWithCovarianceStamped>& _data)
{
  mag_plot_->setData(_data);
}

void PlotTabWidget::setGnssData(const QVector<tobas_msgs::msg::Gnss>& _data)
{
  gnss_plot_->setData(_data);
}

void PlotTabWidget::setBatteryData(const QVector<tobas_msgs::msg::Battery>& _data)
{
  battery_plot_->setData(_data);
}

void PlotTabWidget::setEngineData(const QVector<tobas_msgs::msg::IcePropulsionSystemCommand>& ice_cmd_data)
{
  engine_plot_->setData(ice_cmd_data);
}

void PlotTabWidget::setRotorSpeedData(
  const QVector<tobas_msgs::msg::RotorStateArray>& cur_data,
  const QVector<tobas_msgs::msg::RotorSpeedArray>& tar_data)
{
  rotor_speed_plot_->setData(cur_data, tar_data);
}

void PlotTabWidget::setPropellerPitchData(const QVector<tobas_msgs::msg::IcePropulsionSystemCommand>& ice_cmd_data)
{
  propeller_pitch_plot_->setData(ice_cmd_data);
}

void PlotTabWidget::setSamplingTimeData(const QVector<tobas_msgs::msg::Latency>& _data)
{
  latency_plot_->setSamplingTimeData(_data);
}

void PlotTabWidget::setControlLatencyData(const QVector<tobas_msgs::msg::Latency>& _data)
{
  latency_plot_->setControlLatencyData(_data);
}

void PlotTabWidget::setDisturbanceForceData(const QVector<tobas_kdl_msgs::msg::WrenchStamped>& _data)
{
  dist_force_plot_->setData(_data);
}

void PlotTabWidget::setObserverFeedbackData(const QVector<tobas_debug_msgs::msg::ObserverFeedback>& _data)
{
  obsv_fb_plot_->setData(_data);
}

void PlotTabWidget::setMRControllerFeedbackData(const QVector<tobas_debug_msgs::msg::MultiRotorControllerFeedback>& _data)
{
  mr_ctrl_fb_plot_->setData(_data);
}
}  // namespace log
}  // namespace gui
