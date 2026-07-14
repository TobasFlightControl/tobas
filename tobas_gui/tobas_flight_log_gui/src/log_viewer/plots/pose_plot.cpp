// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_flight_log_gui/log_viewer/plots/pose_plot.hpp"

#include <ranges>

#include <QGridLayout>

#include <tobas_kdl/euler.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_std_tools/unit_conversions.hpp>

namespace tobas
{
namespace gui
{
namespace log
{
PosePlotWidget::PosePlotWidget()
  : cur_curves_{ "Current X [m]",      "Current Y [m]",       "Current Z [m]",
                 "Current Roll [deg]", "Current Pitch [deg]", "Current Yaw [deg]" }
  , tar_curves_{ "Target X [m]",      "Target Y [m]",       "Target Z [m]",
                 "Target Roll [deg]", "Target Pitch [deg]", "Target Yaw [deg]" }
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

void PosePlotWidget::clear()
{
  for (size_t i = 0; i < kNumAxes; ++i) {
    cur_curves_[i].clear();
    tar_curves_[i].clear();
    plots_[i]->replot();
  }
}

void PosePlotWidget::setTimeScale(double t_start, double t_stop)
{
  for (auto& plot : plots_) {
    plot->setAxisScale(QwtPlot::xBottom, t_start, t_stop);
  }
}

void PosePlotWidget::setData(
  const QVector<tobas_msgs::msg::OdometryWithCovarianceStamped>& odom_msgs,
  const QVector<tobas_msgs::msg::OdometryStamped>& setpoint_msgs)
{
  auto ranges = updateCurrentSamples(odom_msgs);
  const auto tar_ranges = updateTargetSamples(setpoint_msgs);

  for (const auto& [range, tar_range] : std::views::zip(ranges, tar_ranges)) {
    range.include(tar_range);
  }

  ranges[kRollAxis].include(-kMinRollPitchScale);
  ranges[kRollAxis].include(kMinRollPitchScale);
  setSharedZeroCenteredVerticalScale(std::span(plots_).subspan(kRollAxis, 1), ranges[kRollAxis]);

  ranges[kPitchAxis].include(-kMinRollPitchScale);
  ranges[kPitchAxis].include(kMinRollPitchScale);
  setSharedZeroCenteredVerticalScale(std::span(plots_).subspan(kPitchAxis, 1), ranges[kPitchAxis]);

  for (auto& plot : plots_) {
    plot->replot();
  }
}

PosePlotWidget::VerticalScaleRanges
PosePlotWidget::updateCurrentSamples(const QVector<tobas_msgs::msg::OdometryWithCovarianceStamped>& odom_msgs)
{
  QVector<double> t_data;
  std::array<QVector<double>, kNumAxes> val_data;
  VerticalScaleRanges ranges;

  for (const auto& odom : odom_msgs) {
    t_data.push_back(ros2::seconds(odom.header.stamp));

    const auto& pos = odom.odom.odom.frame.trans;
    val_data[0].push_back(pos.x);
    val_data[1].push_back(pos.y);
    val_data[2].push_back(pos.z);

    const kdl::Rotation rot(odom.odom.odom.frame.rot.data);
    const auto [roll, pitch, yaw] = rot.getRPY();
    val_data[3].push_back(st::rad2deg(roll));
    val_data[4].push_back(st::rad2deg(pitch));
    val_data[5].push_back(st::rad2deg(yaw));

    for (size_t i = 0; i < kNumAxes; ++i) {
      ranges[i].include(val_data[i].back());
    }
  }

  for (size_t i = 0; i < kNumAxes; ++i) {
    cur_curves_[i].setSamples(t_data, val_data[i]);
  }

  return ranges;
}

PosePlotWidget::VerticalScaleRanges
PosePlotWidget::updateTargetSamples(const QVector<tobas_msgs::msg::OdometryStamped>& setpoint_msgs)
{
  QVector<double> t_data;
  std::array<QVector<double>, kNumAxes> val_data;
  VerticalScaleRanges ranges;
  double roll, pitch, yaw;  // [rad]

  for (const auto& setpoint : setpoint_msgs) {
    t_data.push_back(ros2::seconds(setpoint.header.stamp));

    const auto& pos = setpoint.odom.frame.trans;
    val_data[0].push_back(pos.x);
    val_data[1].push_back(pos.y);
    val_data[2].push_back(pos.z);

    const kdl::Rotation rot(setpoint.odom.frame.rot.data);
    rot.getRPY(roll, pitch, yaw);
    val_data[3].push_back(st::rad2deg(roll));
    val_data[4].push_back(st::rad2deg(pitch));
    val_data[5].push_back(st::rad2deg(yaw));

    for (size_t i = 0; i < kNumAxes; ++i) {
      ranges[i].include(val_data[i].back());
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
