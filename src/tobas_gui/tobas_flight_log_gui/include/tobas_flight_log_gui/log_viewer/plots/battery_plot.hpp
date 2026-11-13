// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_qwt_wrapper/qwt_plot_curve.hpp>

#include <tobas_msgs/msg/battery.hpp>

#include "./utilities/utilities.hpp"

namespace tobas::gui::log
{
class BatteryPlotWidget : public BasePlotWidget
{
  Q_OBJECT

public:
  explicit BatteryPlotWidget();

  void clear() override;
  void setTimeScale(double _t_start, double _t_stop) override;

  void setData(const QVector<tobas_msgs::msg::Battery>& _batt_msgs);

private:
  QwtPlot2* voltage_plot_;
  QwtPlot2* current_plot_;

  qwt::QwtPlotCurveWrapper voltage_curve_;
  qwt::QwtPlotCurveWrapper current_curve_;
};
}  // namespace tobas::gui::log
