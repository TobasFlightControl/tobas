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

  void setSamplingTimeData(const QVector<tobas_msgs::msg::Latency>& sampling_time_msgs);
  void setControlLatencyData(const QVector<tobas_msgs::msg::Latency>& ctrl_latency_msgs);

private:
  QwtPlot2* sampling_time_plot_;
  QwtPlot2* ctrl_latency_plot_;
  qwt::QwtPlotCurveWrapper::SharedPtr sampling_time_curve_;
  qwt::QwtPlotCurveWrapper::SharedPtr ctrl_latency_curve_;
};
}  // namespace log
}  // namespace gui
