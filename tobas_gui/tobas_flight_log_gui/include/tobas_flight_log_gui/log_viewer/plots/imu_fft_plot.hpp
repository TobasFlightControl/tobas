// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <eigen3/unsupported/Eigen/FFT>

#include <tobas_msgs/msg/imu.hpp>

#include "./common.hpp"

namespace tobas
{
namespace gui
{
namespace log
{
class ImuFftPlotWidget : public BasePlotWidget
{
  Q_OBJECT

  static constexpr size_t kNumAxes = 6;

public:
  explicit ImuFftPlotWidget();

  void clear() override;
  void setTimeScale(double t_start, double t_stop) override;

  void setData(const QVector<tobas_msgs::msg::Imu>& raw_msgs, const QVector<tobas_msgs::msg::Imu>& filt_msgs);

private:
  std::array<Eigen::FFT<double>, kNumAxes> raw_ffts_, filt_ffts_;

  std::array<QwtPlot2*, kNumAxes> plots_;
  std::array<qwt::QwtPlotCurveWrapper, kNumAxes> raw_curves_;
  std::array<qwt::QwtPlotCurveWrapper, kNumAxes> filt_curves_;

  static void updateSamples(
    const QVector<tobas_msgs::msg::Imu>& msgs,
    std::array<Eigen::FFT<double>, kNumAxes>& ffts,
    std::array<qwt::QwtPlotCurveWrapper, kNumAxes>& curves);
};
}  // namespace log
}  // namespace gui
}  // namespace tobas
