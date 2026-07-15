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

  static constexpr int kSamplingTimeMin = 500;     // [us]
  static constexpr int kSamplingTimeMax = 2000;    // [us]
  static constexpr int kControlLatencyMin = 0;     // [us]
  static constexpr int kControlLatencyMax = 1500;  // [us]
  static constexpr int kBinWidth = 10;             // [us]

public:
  explicit LatencyPlotWidget();

  void clear() override;
  void setTimeScale(double t_start, double t_stop) override;

  void setData(
    const QVector<tobas_msgs::msg::Latency>& sampling_time_msgs,
    const QVector<tobas_msgs::msg::Latency>& ctrl_latency_msgs);

private:
  QwtPlot2* sampling_time_plot_;
  qwt::QwtPlotCurveWrapper sampling_time_curve_;

  QwtPlot2* ctrl_latency_plot_;
  qwt::QwtPlotCurveWrapper ctrl_latency_curve_;

  QwtPlot2* sampling_time_hist_plot_;
  qwt::QwtPlotHistogramWrapper sampling_time_hist_data_;

  QwtPlot2* ctrl_latency_hist_plot_;
  qwt::QwtPlotHistogramWrapper ctrl_latency_hist_data_;

  void setSamplingTimeData(const QVector<tobas_msgs::msg::Latency>& msgs);
  void setControlLatencyData(const QVector<tobas_msgs::msg::Latency>& msgs);
};
}  // namespace log
}  // namespace gui
}  // namespace tobas
