// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_qwt_wrapper/qwt_plot_curve.hpp>

#include <tobas_msgs/msg/pwm_array.hpp>

#include "./utilities/utilities.hpp"

namespace tobas::gui::log
{
class PwmPlotWidget : public BasePlotWidget
{
  Q_OBJECT

public:
  explicit PwmPlotWidget();

  void clear() override;
  void setTimeScale(double _t_start, double _t_stop) override;

  void setData(const QVector<tobas_msgs::msg::PwmArray>& _msgs);

private:
  QVector<QwtPlot2*> plots_;
  QVector<qwt::QwtPlotCurveWrapper> curves_;
};
}  // namespace tobas::gui::log
