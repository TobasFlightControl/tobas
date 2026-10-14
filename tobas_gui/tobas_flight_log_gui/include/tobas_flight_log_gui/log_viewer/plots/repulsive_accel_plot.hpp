// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_qwt_wrapper/qwt_plot_curve.hpp>

#include <tobas_msgs/msg/repulsive_acceleration.hpp>

#include "./utilities/utilities.hpp"

namespace tobas::gui::log
{
class RepulsiveAccelPlotWidget : public BasePlotWidget
{
  Q_OBJECT

  static constexpr size_t kNumAxes = 3;

public:
  explicit RepulsiveAccelPlotWidget();

  void clear() override;
  void setTimeScale(double _t_start, double _t_stop) override;

  void setData(const QVector<tobas_msgs::msg::RepulsiveAcceleration>& _msgs);

private:
  std::array<QwtPlot2*, kNumAxes> plots_;
  std::array<qwt::QwtPlotCurveWrapper, kNumAxes> curves_;
};
}  // namespace tobas::gui::log
