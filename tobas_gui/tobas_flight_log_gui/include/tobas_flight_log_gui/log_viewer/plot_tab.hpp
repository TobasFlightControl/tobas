#pragma once

#include <tobas_qt_tools/widgets/tab_widget.hpp>

#include "./imu_plot.hpp"

namespace gui
{
namespace log
{
class PlotTabWidget : public qt::TabWidget
{
  Q_OBJECT

  using self = PlotTabWidget;
  using super = qt::TabWidget;

public:
  explicit PlotTabWidget();

  void setTimeScale(double t_start, double t_stop);

  void setImuData(const QVector<tobas_msgs::msg::ImuWithCovarianceStamped>& _data);

private:
  ImuPlotWidget* imu_plot_;
};
}  // namespace log
}  // namespace gui
