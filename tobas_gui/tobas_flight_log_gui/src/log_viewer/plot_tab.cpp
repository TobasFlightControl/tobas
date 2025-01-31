#include "tobas_flight_log_gui/log_viewer/plot_tab.hpp"

namespace gui
{
namespace log
{
PlotTabWidget::PlotTabWidget()
{
  imu_plot_ = new ImuPlotWidget();
  mag_plot_ = new MagPlotWidget();
  // TODO

  addTab(imu_plot_, "IMU");
  addTab(mag_plot_, "Mag");
  // TODO
}

void PlotTabWidget::setTimeScale(double t_start, double t_stop)
{
  imu_plot_->setTimeScale(t_start, t_stop);
  mag_plot_->setTimeScale(t_start, t_stop);
  // TODO
}

void PlotTabWidget::setImuData(const QVector<tobas_msgs::msg::ImuWithCovarianceStamped>& _data)
{
  imu_plot_->setData(_data);
}

void PlotTabWidget::setMagData(const QVector<tobas_msgs::msg::MagneticFieldWithCovarianceStamped>& _data)
{
  mag_plot_->setData(_data);
}
}  // namespace log
}  // namespace gui
