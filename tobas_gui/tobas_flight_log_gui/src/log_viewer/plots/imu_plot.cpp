// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_flight_log_gui/log_viewer/plots/imu_plot.hpp"

#include <QGridLayout>

#include <tobas_ros2_tools/time.hpp>

namespace tobas
{
namespace gui
{
namespace log
{
ImuPlotWidget::ImuPlotWidget()
  : raw_curves_{ "Raw Accel X [m/s²]", "Raw Accel Y [m/s²]", "Raw Accel Z [m/s²]",
                 "Raw Gyro X [rad/s]", "Raw Gyro Y [rad/s]", "Raw Gyro Z [rad/s]" }
  , filt_curves_{ "Filtered Accel X [m/s²]", "Filtered Accel Y [m/s²]", "Filtered Accel Z [m/s²]",
                  "Filtered Gyro X [rad/s]", "Filtered Gyro Y [rad/s]", "Filtered Gyro Z [rad/s]" }
{
  const auto grid = new QGridLayout();
  setLayout(grid);

  for (size_t i = 0; i < kNumAxes; ++i) {
    plots_[i] = new QwtPlot2();
    plots_[i]->setAxisNoLabel(QwtPlot::xBottom);
    grid->addWidget(plots_[i], i % 3, i / 3, 1, 1);

    raw_curves_[i].setPen(kRawValueColor, kLineWidth);
    raw_curves_[i].attach(plots_[i]);

    filt_curves_[i].setPen(kFilteredValueColor, kLineWidth);
    filt_curves_[i].attach(plots_[i]);
  }
}

void ImuPlotWidget::clear()
{
  for (size_t i = 0; i < kNumAxes; ++i) {
    raw_curves_[i].clear();
    filt_curves_[i].clear();
    plots_[i]->replot();
  }
}

void ImuPlotWidget::setTimeScale(double t_start, double t_stop)
{
  for (auto& plot : plots_) {
    plot->setAxisScale(QwtPlot::xBottom, t_start, t_stop);
  }
}

void ImuPlotWidget::setData(const QVector<tobas_msgs::msg::Imu>& raw_msgs, const QVector<tobas_msgs::msg::Imu>& filt_msgs)
{
  auto ranges = updateSamples(raw_msgs, raw_curves_);
  const auto filt_ranges = updateSamples(filt_msgs, filt_curves_);

  for (size_t group = 0; group < kNumGroups; ++group) {
    ranges[group].include(filt_ranges[group]);
    setSharedZeroCenteredVerticalScale(
      std::span(plots_).subspan(group * kNumAxesPerGroup, kNumAxesPerGroup), ranges[group]);
  }

  for (auto& plot : plots_) {
    plot->replot();
  }
}

ImuPlotWidget::ValueRanges ImuPlotWidget::updateSamples(
  const QVector<tobas_msgs::msg::Imu>& msgs,
  std::array<qwt::QwtPlotCurveWrapper, kNumAxes>& curves)
{
  QVector<double> t_data;
  std::array<QVector<double>, kNumAxes> val_data;
  ValueRanges ranges;

  for (const auto& msg : msgs) {
    t_data.push_back(ros2::seconds(msg.header.stamp));

    const auto& accel = msg.accel;
    val_data[0].push_back(accel.x);
    val_data[1].push_back(accel.y);
    val_data[2].push_back(accel.z);

    const auto& gyro = msg.gyro;
    val_data[3].push_back(gyro.x);
    val_data[4].push_back(gyro.y);
    val_data[5].push_back(gyro.z);

    for (size_t i = 0; i < kNumAxes; ++i) {
      ranges[i / kNumAxesPerGroup].include(val_data[i].back());
    }
  }

  for (size_t i = 0; i < kNumAxes; ++i) {
    curves[i].setSamples(t_data, val_data[i]);
  }

  return ranges;
}
}  // namespace log
}  // namespace gui
}  // namespace tobas
