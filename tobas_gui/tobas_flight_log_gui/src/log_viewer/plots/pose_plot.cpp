// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_flight_log_gui/log_viewer/plots/pose_plot.hpp"

#include <array>
#include <cmath>
#include <optional>

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
namespace
{
/* Maps an angle to the closest equivalent value on the previous sample's continuous branch. */
double unwrapAngle(double angle, std::optional<double>& previous_angle)
{
  if (!std::isfinite(angle)) {
    return angle;
  }
  if (previous_angle) {
    angle += 360.0 * std::round((*previous_angle - angle) / 360.0);
  }
  previous_angle = angle;
  return angle;
}
}  // namespace

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
  const auto ranges = updateCurrentSamples(odom_msgs);
  const auto tar_ranges = updateTargetSamples(setpoint_msgs, ranges);

  constexpr double kMinPositionScale = 1.0;  // [m]
  constexpr double kMinAngleScale = 4.0;     // [deg]
  for (size_t i = 0; i < kNumAxes; ++i) {
    const auto minimum_half_range = i <= kZAxis ? kMinPositionScale : kMinAngleScale;
    setTargetCenteredVerticalScale(*plots_[i], ranges[i], tar_ranges[i], minimum_half_range);
  }

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
  std::array<std::optional<double>, 3> previous_angles;
  double roll, pitch, yaw;  // [rad]

  for (const auto& odom : odom_msgs) {
    t_data.push_back(ros2::seconds(odom.header.stamp));

    const auto& pos = odom.odom.odom.frame.trans;
    val_data[0].push_back(pos.x);
    val_data[1].push_back(pos.y);
    val_data[2].push_back(pos.z);

    const kdl::Rotation rot(odom.odom.odom.frame.rot.data);
    rot.getRPY(roll, pitch, yaw);
    const std::array angles{ st::rad2deg(roll), st::rad2deg(pitch), st::rad2deg(yaw) };  // [deg]
    for (size_t i = 0; i < angles.size(); ++i) {
      val_data[kRollAxis + i].push_back(unwrapAngle(angles[i], previous_angles[i]));
    }

    for (size_t i = 0; i < kNumAxes; ++i) {
      ranges[i].include(val_data[i].back());
    }
  }

  for (size_t i = 0; i < kNumAxes; ++i) {
    cur_curves_[i].setSamples(t_data, val_data[i]);
  }

  return ranges;
}

PosePlotWidget::VerticalScaleRanges PosePlotWidget::updateTargetSamples(
  const QVector<tobas_msgs::msg::OdometryStamped>& setpoint_msgs,
  const VerticalScaleRanges& current_ranges)
{
  QVector<double> t_data;
  std::array<QVector<double>, kNumAxes> val_data;
  VerticalScaleRanges ranges;
  std::array<std::optional<double>, 3> previous_angles;
  double roll, pitch, yaw;  // [rad]

  for (size_t i = 0; i < 3; ++i) {
    const auto& current_range = current_ranges[kRollAxis + i];
    if (!current_range.empty()) {
      previous_angles[i] = current_range.center();
    }
  }

  for (const auto& setpoint : setpoint_msgs) {
    t_data.push_back(ros2::seconds(setpoint.header.stamp));

    const auto& pos = setpoint.odom.frame.trans;
    val_data[0].push_back(pos.x);
    val_data[1].push_back(pos.y);
    val_data[2].push_back(pos.z);

    const kdl::Rotation rot(setpoint.odom.frame.rot.data);
    rot.getRPY(roll, pitch, yaw);
    const std::array angles{ st::rad2deg(roll), st::rad2deg(pitch), st::rad2deg(yaw) };  // [deg]
    for (size_t i = 0; i < angles.size(); ++i) {
      val_data[kRollAxis + i].push_back(unwrapAngle(angles[i], previous_angles[i]));
    }

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
