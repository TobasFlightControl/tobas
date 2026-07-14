// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_flight_log_gui/log_viewer/plots/twist_plot.hpp"

#include <QGridLayout>

#include <tobas_ros2_tools/time.hpp>

namespace tobas
{
namespace gui
{
namespace log
{
TwistPlotWidget::TwistPlotWidget()
  : cur_curves_{ "Current Linear Velocity X [m/s]",    "Current Linear Velocity Y [m/s]",
                 "Current Linear Velocity Z [m/s]",    "Current Angular Velocity X [rad/s]",
                 "Current Angular Velocity Y [rad/s]", "Current Angular Velocity Z [rad/s]" }
  , tar_curves_{ "Target Linear Velocity X [m/s]",    "Target Linear Velocity Y [m/s]",
                 "Target Linear Velocity Z [m/s]",    "Target Angular Velocity X [rad/s]",
                 "Target Angular Velocity Y [rad/s]", "Target Angular Velocity Z [rad/s]" }
{
  const auto grid = new QGridLayout();
  setLayout(grid);

  for (size_t i = 0; i < kNumAxes; ++i) {
    plots_[i] = new QwtPlot2();
    plots_[i]->setAxisNoLabel(QwtPlot::xBottom);
    grid->addWidget(plots_[i], i % 3, i / 3, 1, 1);

    cur_curves_[i].setPen(kCurrentValueColor, kLineWidth);
    cur_curves_[i].attach(plots_[i]);

    tar_curves_[i].setPen(kTargetValueColor, kLineWidth);
    tar_curves_[i].attach(plots_[i]);
  }
}

void TwistPlotWidget::clear()
{
  for (size_t i = 0; i < kNumAxes; ++i) {
    cur_curves_[i].clear();
    tar_curves_[i].clear();
    plots_[i]->replot();
  }
}

void TwistPlotWidget::setTimeScale(double t_start, double t_stop)
{
  for (auto& plot : plots_) {
    plot->setAxisScale(QwtPlot::xBottom, t_start, t_stop);
  }
}

void TwistPlotWidget::setData(
  const QVector<tobas_msgs::msg::OdometryWithCovarianceStamped>& odom_msgs,
  const QVector<tobas_msgs::msg::OdometryStamped>& setpoint_msgs)
{
  auto ranges = updateCurrentSamples(odom_msgs);
  const auto tar_ranges = updateTargetSamples(setpoint_msgs);

  for (size_t group = 0; group < kNumGroups; ++group) {
    ranges[group].include(tar_ranges[group]);
    setSharedVerticalScale(std::span(plots_).subspan(group * kNumAxesPerGroup, kNumAxesPerGroup), ranges[group]);
  }

  for (auto& plot : plots_) {
    plot->replot();
  }
}

TwistPlotWidget::ValueRanges
TwistPlotWidget::updateCurrentSamples(const QVector<tobas_msgs::msg::OdometryWithCovarianceStamped>& odom_msgs)
{
  QVector<double> t_data;
  std::array<QVector<double>, kNumAxes> val_data;
  ValueRanges ranges;

  for (const auto& odom : odom_msgs) {
    t_data.push_back(ros2::seconds(odom.header.stamp));

    const auto& lin_vel = odom.odom.odom.twist.linear;
    val_data[0].push_back(lin_vel.x);
    val_data[1].push_back(lin_vel.y);
    val_data[2].push_back(lin_vel.z);

    const auto& ang_vel = odom.odom.odom.twist.angular;
    val_data[3].push_back(ang_vel.x);
    val_data[4].push_back(ang_vel.y);
    val_data[5].push_back(ang_vel.z);

    for (size_t i = 0; i < kNumAxes; ++i) {
      ranges[i / kNumAxesPerGroup].include(val_data[i].back());
    }
  }

  for (size_t i = 0; i < kNumAxes; ++i) {
    cur_curves_[i].setSamples(t_data, val_data[i]);
  }

  return ranges;
}

TwistPlotWidget::ValueRanges
TwistPlotWidget::updateTargetSamples(const QVector<tobas_msgs::msg::OdometryStamped>& setpoint_msgs)
{
  QVector<double> t_data;
  std::array<QVector<double>, kNumAxes> val_data;
  ValueRanges ranges;

  for (const auto& setpoint : setpoint_msgs) {
    t_data.push_back(ros2::seconds(setpoint.header.stamp));

    const auto& lin_vel = setpoint.odom.twist.linear;
    val_data[0].push_back(lin_vel.x);
    val_data[1].push_back(lin_vel.y);
    val_data[2].push_back(lin_vel.z);

    const auto& ang_vel = setpoint.odom.twist.angular;
    val_data[3].push_back(ang_vel.x);
    val_data[4].push_back(ang_vel.y);
    val_data[5].push_back(ang_vel.z);

    for (size_t i = 0; i < kNumAxes; ++i) {
      ranges[i / kNumAxesPerGroup].include(val_data[i].back());
    }
  }

  for (size_t i = 0; i < kNumAxes; ++i) {
    tar_curves_[i].setSamples(t_data, val_data[i]);
  }

  return ranges;
}
}  // namespace log
}  // namespace gui
}  // namespace tobas
