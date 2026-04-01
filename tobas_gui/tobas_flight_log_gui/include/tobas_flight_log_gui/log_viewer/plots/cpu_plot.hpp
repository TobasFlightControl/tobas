// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_msgs/msg/cpu.hpp>

#include "./common.hpp"

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
  void setTimeScale(double t_start, double t_stop) override;

  void setData(const QVector<tobas_msgs::msg::Cpu>& msgs);

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
