// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_msgs/msg/imu.hpp>

#include "./utilities/utilities.hpp"

namespace tobas
{
namespace gui
{
namespace log
{
class ImuPlotWidget : public BasePlotWidget
{
  Q_OBJECT

  static constexpr size_t kNumAxesPerGroup = 3;
  static constexpr size_t kNumGroups = 2;
  static constexpr size_t kNumAxes = kNumAxesPerGroup * kNumGroups;

  using ValueRanges = std::array<VerticalScaleRange, kNumGroups>;

public:
  explicit ImuPlotWidget();

  void clear() override;
  void setTimeScale(double t_start, double t_stop) override;

  void setData(const QVector<tobas_msgs::msg::Imu>& raw_msgs, const QVector<tobas_msgs::msg::Imu>& filt_msgs);

private:
  std::array<QwtPlot2*, kNumAxes> plots_;
  std::array<qwt::QwtPlotCurveWrapper, kNumAxes> raw_curves_;
  std::array<qwt::QwtPlotCurveWrapper, kNumAxes> filt_curves_;

  static ValueRanges
  updateSamples(const QVector<tobas_msgs::msg::Imu>& msgs, std::array<qwt::QwtPlotCurveWrapper, kNumAxes>& curves);
};
}  // namespace log
}  // namespace gui
}  // namespace tobas
