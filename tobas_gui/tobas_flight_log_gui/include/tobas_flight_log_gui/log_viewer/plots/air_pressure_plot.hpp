// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_msgs/msg/fluid_pressure.hpp>

#include "./common.hpp"

namespace tobas
{
namespace gui
{
namespace log
{
class AirPressurePlotWidget : public BasePlotWidget
{
  Q_OBJECT

public:
  explicit AirPressurePlotWidget();

  void clear() override;
  void setTimeScale(double t_start, double t_stop) override;

  void setData(const QVector<tobas_msgs::msg::FluidPressure>& msgs);

private:
  QwtPlot2* plot_;
  qwt::QwtPlotCurveWrapper curve_;
};
}  // namespace log
}  // namespace gui
}  // namespace tobas
