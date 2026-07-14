// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_msgs/msg/ice_propulsion_system_command.hpp>

#include "./utilities/utilities.hpp"

namespace tobas
{
namespace gui
{
namespace log
{
class EnginePlotWidget : public BasePlotWidget
{
  Q_OBJECT

public:
  explicit EnginePlotWidget();

  void clear() override;
  void setTimeScale(double t_start, double t_stop) override;

  void setData(const QVector<tobas_msgs::msg::IcePropulsionSystemCommand>& msgs);

private:
  QwtPlot2* throttle_plot_;
  qwt::QwtPlotCurveWrapper throttle_curve_;
};
}  // namespace log
}  // namespace gui
}  // namespace tobas
