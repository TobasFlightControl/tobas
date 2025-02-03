#include "tobas_flight_log_gui/log_viewer/plot_tab.hpp"

namespace gui
{
namespace log
{
PlotTabWidget::PlotTabWidget()
{
  pose_plot_ = new PosePlotWidget();
  twist_plot_ = new TwistPlotWidget();
  imu_plot_ = new ImuPlotWidget();
  mag_plot_ = new MagPlotWidget();
  gps_plot_ = new GpsPlotWidget();
  batt_plot_ = new BatteryPlotWidget();
  latency_plot_ = new LatencyPlotWidget();
  dist_force_plot_ = new DisturbanceForcePlotWidget();
  // TODO

  addTab(pose_plot_, "Pose");
  addTab(twist_plot_, "Twist");
  addTab(imu_plot_, "IMU");
  addTab(mag_plot_, "Mag");
  addTab(gps_plot_, "GPS");
  addTab(batt_plot_, "Battery");
  addTab(latency_plot_, "Latency");
  addTab(dist_force_plot_, "Dist Force");
  // TODO
}

void PlotTabWidget::setTimeScale(double t_start, double t_stop)
{
  pose_plot_->setTimeScale(t_start, t_stop);
  twist_plot_->setTimeScale(t_start, t_stop);
  imu_plot_->setTimeScale(t_start, t_stop);
  mag_plot_->setTimeScale(t_start, t_stop);
  gps_plot_->setTimeScale(t_start, t_stop);
  batt_plot_->setTimeScale(t_start, t_stop);
  latency_plot_->setTimeScale(t_start, t_stop);
  dist_force_plot_->setTimeScale(t_start, t_stop);
  // TODO
}

void PlotTabWidget::setPoseData(const QVector<tobas_msgs::msg::Odometry>& _data)
{
  pose_plot_->setData(_data);
}

void PlotTabWidget::setTwistData(const QVector<tobas_msgs::msg::Odometry>& _data)
{
  twist_plot_->setData(_data);
}

void PlotTabWidget::setImuData(const QVector<tobas_msgs::msg::ImuWithCovarianceStamped>& _data)
{
  imu_plot_->setData(_data);
}

void PlotTabWidget::setMagData(const QVector<tobas_msgs::msg::MagneticFieldWithCovarianceStamped>& _data)
{
  mag_plot_->setData(_data);
}

void PlotTabWidget::setGpsData(const QVector<tobas_msgs::msg::Gps>& _data)
{
  gps_plot_->setData(_data);
}

void PlotTabWidget::setBatteryData(const QVector<tobas_msgs::msg::Battery>& _data)
{
  batt_plot_->setData(_data);
}

void PlotTabWidget::setLatencyData(const QVector<tobas_msgs::msg::Latency>& _data)
{
  latency_plot_->setData(_data);
}

void PlotTabWidget::setDisturbanceForceData(const QVector<tobas_kdl_msgs::msg::WrenchStamped>& _data)
{
  dist_force_plot_->setData(_data);
}
}  // namespace log
}  // namespace gui
