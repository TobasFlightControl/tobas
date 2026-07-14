// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <array>

#include <tobas_qwt_wrapper/qwt_plot_curve.hpp>

#include <tobas_msgs/msg/odometry_stamped.hpp>
#include <tobas_msgs/msg/odometry_with_covariance_stamped.hpp>

#include "./utilities/utilities.hpp"

namespace tobas
{
namespace gui
{
namespace log
{
class PosePlotWidget : public BasePlotWidget
{
  Q_OBJECT

  static constexpr size_t kXAxis = 0;
  static constexpr size_t kYAxis = kXAxis + 1;
  static constexpr size_t kZAxis = kYAxis + 1;
  static constexpr size_t kRollAxis = kZAxis + 1;
  static constexpr size_t kPitchAxis = kRollAxis + 1;
  static constexpr size_t kYawAxis = kPitchAxis + 1;
  static constexpr size_t kNumAxes = kYawAxis + 1;

  static constexpr double kMinPositionScale = 1.0;  // [m]
  static constexpr double kMinAngleScale = 4.0;     // [deg]

  using VerticalScaleRanges = std::array<VerticalScaleRange, kNumAxes>;

public:
  explicit PosePlotWidget();

  void clear() override;
  void setTimeScale(double t_start, double t_stop) override;

  void setData(
    const QVector<tobas_msgs::msg::OdometryWithCovarianceStamped>& odom_msgs,
    const QVector<tobas_msgs::msg::OdometryStamped>& setpoint_msgs);

private:
  std::array<QwtPlot2*, kNumAxes> plots_;

  std::array<qwt::QwtPlotCurveWrapper, kNumAxes> cur_curves_;
  std::array<qwt::QwtPlotCurveWrapper, kNumAxes> tar_curves_;

  VerticalScaleRanges updateCurrentSamples(const QVector<tobas_msgs::msg::OdometryWithCovarianceStamped>& odom_msgs);
  VerticalScaleRanges updateTargetSamples(const QVector<tobas_msgs::msg::OdometryStamped>& setpoint_msgs);
};
}  // namespace log
}  // namespace gui
}  // namespace tobas
