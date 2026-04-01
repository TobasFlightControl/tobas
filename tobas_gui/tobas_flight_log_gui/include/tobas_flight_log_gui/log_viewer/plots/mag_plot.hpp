// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_msgs/msg/magnetic_field.hpp>

#include "./common.hpp"

namespace tobas
{
namespace gui
{
namespace log
{
class MagPlotWidget : public BasePlotWidget
{
  Q_OBJECT

  static constexpr size_t kNumAxes = 3;

public:
  explicit MagPlotWidget();

  void clear() override;
  void setTimeScale(double t_start, double t_stop) override;

  void setData(const QVector<tobas_msgs::msg::MagneticField>& mag_msgs);

private:
  std::array<QwtPlot2*, 3> mag_plots_;
  std::array<qwt::QwtPlotCurveWrapper, 3> mag_curves_;
};
}  // namespace log
}  // namespace gui
}  // namespace tobas
