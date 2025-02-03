#pragma once

#include <tobas_msgs/msg/battery.hpp>

#include "./common.hpp"

namespace gui
{
namespace log
{
class BatteryPlotWidget : public QWidget
{
  Q_OBJECT

public:
  explicit BatteryPlotWidget();

  void setTimeScale(double t_start, double t_stop);
  void setData(const QVector<tobas_msgs::msg::Battery>& batt_msgs);

private:
  QwtPlot2* voltage_plot_;
  QwtPlot2* current_plot_;

  QwtPlotCurve* voltage_curve_;
  QwtPlotCurve* current_curve_;
};
}  // namespace log
}  // namespace gui
