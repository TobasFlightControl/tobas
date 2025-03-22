#pragma once

#include <tobas_msgs/msg/ice_propulsion_system_command.hpp>

#include "./common.hpp"

namespace gui
{
namespace log
{
class EnginePlotWidget : public QWidget
{
  Q_OBJECT

public:
  explicit EnginePlotWidget();

  void setTimeScale(double t_start, double t_stop);
  void setData(const QVector<tobas_msgs::msg::IcePropulsionSystemCommand>& msgs);

private:
  QwtPlot2* throttle_plot_;
  qwt::QwtPlotCurveWrapper throttle_curve_;
};
}  // namespace log
}  // namespace gui
