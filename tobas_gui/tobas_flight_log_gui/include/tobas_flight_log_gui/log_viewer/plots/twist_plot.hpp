// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <array>

#include <tobas_msgs/msg/odometry_stamped.hpp>
#include <tobas_msgs/msg/odometry_with_covariance_stamped.hpp>

#include "./common.hpp"

namespace tobas
{
namespace gui
{
namespace log
{
class TwistPlotWidget : public BasePlotWidget
{
  Q_OBJECT

  static constexpr size_t kNumAxesPerGroup = 3;
  static constexpr size_t kNumGroups = 2;
  static constexpr size_t kNumAxes = kNumAxesPerGroup * kNumGroups;

  using ValueRanges = std::array<VerticalScaleRange, kNumGroups>;

public:
  explicit TwistPlotWidget();

  void clear() override;
  void setTimeScale(double t_start, double t_stop) override;

  void setData(
    const QVector<tobas_msgs::msg::OdometryWithCovarianceStamped>& odom_msgs,
    const QVector<tobas_msgs::msg::OdometryStamped>& setpoint_msgs);

private:
  std::array<QwtPlot2*, kNumAxes> plots_;

  std::array<qwt::QwtPlotCurveWrapper, kNumAxes> cur_curves_;
  std::array<qwt::QwtPlotCurveWrapper, kNumAxes> tar_curves_;

  ValueRanges updateCurrentSamples(const QVector<tobas_msgs::msg::OdometryWithCovarianceStamped>& odom_msgs);
  ValueRanges updateTargetSamples(const QVector<tobas_msgs::msg::OdometryStamped>& setpoint_msgs);
};
}  // namespace log
}  // namespace gui
}  // namespace tobas
