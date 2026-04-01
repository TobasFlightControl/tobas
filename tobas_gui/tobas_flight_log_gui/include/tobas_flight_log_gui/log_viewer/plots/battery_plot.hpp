// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_msgs/msg/battery.hpp>

#include "./common.hpp"

namespace tobas
{
namespace gui
{
namespace log
{
class BatteryPlotWidget : public BasePlotWidget
{
  Q_OBJECT

public:
  explicit BatteryPlotWidget();

  void clear() override;
  void setTimeScale(double t_start, double t_stop) override;

  void setData(const QVector<tobas_msgs::msg::Battery>& batt_msgs);

private:
  QwtPlot2* voltage_plot_;
  QwtPlot2* current_plot_;

  qwt::QwtPlotCurveWrapper voltage_curve_;
  qwt::QwtPlotCurveWrapper current_curve_;
};
}  // namespace log
}  // namespace gui
}  // namespace tobas
