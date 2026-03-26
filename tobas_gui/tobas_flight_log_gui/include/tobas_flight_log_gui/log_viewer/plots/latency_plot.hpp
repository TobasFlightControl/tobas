#pragma once

#include <tobas_msgs/msg/latency.hpp>

#include "./common.hpp"

namespace tobas
{
namespace gui
{
namespace log
{
class LatencyPlotWidget : public BasePlotWidget
{
  Q_OBJECT

public:
  explicit LatencyPlotWidget();

  void clear() override;
  void setTimeScale(double t_start, double t_stop) override;

  void setData(
    const QVector<tobas_msgs::msg::Latency>& sampling_time_msgs,
    const QVector<tobas_msgs::msg::Latency>& ctrl_latency_msgs);

private:
  QwtPlot2* sampling_time_plot_;
  QwtPlot2* ctrl_latency_plot_;
  qwt::QwtPlotCurveWrapper sampling_time_curve_;
  qwt::QwtPlotCurveWrapper ctrl_latency_curve_;

  void setSamplingTimeData(const QVector<tobas_msgs::msg::Latency>& msgs);
  void setControlLatencyData(const QVector<tobas_msgs::msg::Latency>& msgs);
};
}  // namespace log
}  // namespace gui
}  // namespace tobas
