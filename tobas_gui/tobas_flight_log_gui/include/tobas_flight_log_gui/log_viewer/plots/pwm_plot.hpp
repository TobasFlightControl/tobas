// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_qwt_wrapper/qwt_plot_curve.hpp>

#include <tobas_msgs/msg/pwm_array.hpp>

#include "./utilities/utilities.hpp"

namespace tobas
{
namespace gui
{
namespace log
{
class PwmPlotWidget : public BasePlotWidget
{
  Q_OBJECT

  static constexpr int kPwmChannels = 8;

public:
  explicit PwmPlotWidget();

  void clear() override;
  void setTimeScale(double t_start, double t_stop) override;

  void setData(const QVector<tobas_msgs::msg::PwmArray>& msgs);

private:
  QVector<QwtPlot2*> plots_;
  QVector<qwt::QwtPlotCurveWrapper> curves_;
};
}  // namespace log
}  // namespace gui
}  // namespace tobas
