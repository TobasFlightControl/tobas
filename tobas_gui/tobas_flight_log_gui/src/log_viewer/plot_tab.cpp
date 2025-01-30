#include "tobas_flight_log_gui/log_viewer/plot_tab.hpp"

namespace gui
{
namespace log
{
PlotTabWidget::PlotTabWidget()
{
  imu_plot_ = new ImuPlotWidget();

  addTab(imu_plot_, "IMU");
}
}  // namespace log
}  // namespace gui
