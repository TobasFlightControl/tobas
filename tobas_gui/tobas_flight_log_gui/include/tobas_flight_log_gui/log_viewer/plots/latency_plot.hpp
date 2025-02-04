#pragma once

#include <tobas_msgs/msg/latency.hpp>

#include "./common.hpp"

namespace gui
{
namespace log
{
class LatencyPlotWidget : public QWidget
{
  Q_OBJECT

public:
  explicit LatencyPlotWidget();

  void setTimeScale(double t_start, double t_stop);
  void setData(const QVector<tobas_msgs::msg::Latency>& latency_msgs);

private:
  QwtPlot2* plot_;
  qwt::QwtPlotCurveWrapper* curve_;
};
}  // namespace log
}  // namespace gui
