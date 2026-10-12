// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_qwt_wrapper/qwt_plot_curve.hpp>

#include <tobas_msgs/msg/cpu.hpp>

#include "./utilities/utilities.hpp"

namespace tobas
{
namespace gui
{
namespace log
{
class CpuPlotWidget : public BasePlotWidget
{
  Q_OBJECT

public:
  explicit CpuPlotWidget();

  void clear() override;
  void setTimeScale(double _t_start, double _t_stop) override;

  void setData(const QVector<tobas_msgs::msg::Cpu>& _msgs);

private:
  QwtPlot2* freq_plot_;
  QwtPlot2* temp_plot_;
  QwtPlot2* load_plot_;

  qwt::QwtPlotCurveWrapper freq_curve_;
  qwt::QwtPlotCurveWrapper temp_curve_;
  qwt::QwtPlotCurveWrapper load_curve_;
};
}  // namespace log
}  // namespace gui
}  // namespace tobas
