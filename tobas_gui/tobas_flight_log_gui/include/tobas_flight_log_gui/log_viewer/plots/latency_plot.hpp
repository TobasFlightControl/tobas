// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_qwt_wrapper/qwt_plot_curve.hpp>
#include <tobas_qwt_wrapper/qwt_plot_histogram.hpp>

#include <tobas_msgs/msg/latency.hpp>

#include "./utilities/utilities.hpp"

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
  void setTimeScale(double _t_start, double _t_stop) override;

  void setData(
    const QVector<tobas_msgs::msg::Latency>& _sampling_time_msgs,
    const QVector<tobas_msgs::msg::Latency>& _ctrl_latency_msgs);

private:
  QwtPlot2* sampling_time_plot_;
  qwt::QwtPlotCurveWrapper sampling_time_curve_;

  QwtPlot2* ctrl_latency_plot_;
  qwt::QwtPlotCurveWrapper ctrl_latency_curve_;

  QwtPlot2* sampling_time_hist_plot_;
  qwt::QwtPlotHistogramWrapper sampling_time_hist_data_;

  QwtPlot2* ctrl_latency_hist_plot_;
  qwt::QwtPlotHistogramWrapper ctrl_latency_hist_data_;

  void setSamplingTimeData(const QVector<tobas_msgs::msg::Latency>& _msgs);
  void setControlLatencyData(const QVector<tobas_msgs::msg::Latency>& _msgs);
};
}  // namespace log
}  // namespace gui
}  // namespace tobas
