// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_kdl_msgs/msg/wrench_stamped.hpp>

#include "./common.hpp"

namespace tobas
{
namespace gui
{
namespace log
{
class DisturbanceForcePlotWidget : public BasePlotWidget
{
  Q_OBJECT

  static constexpr size_t kNumAxesPerGroup = 3;
  static constexpr size_t kNumGroups = 2;
  static constexpr size_t kNumAxes = kNumAxesPerGroup * kNumGroups;

public:
  explicit DisturbanceForcePlotWidget();

  void clear() override;
  void setTimeScale(double t_start, double t_stop) override;

  void setData(const QVector<tobas_kdl_msgs::msg::WrenchStamped>& msgs);

private:
  std::array<QwtPlot2*, kNumAxes> plots_;
  std::array<qwt::QwtPlotCurveWrapper, kNumAxes> curves_;
};
}  // namespace log
}  // namespace gui
}  // namespace tobas
