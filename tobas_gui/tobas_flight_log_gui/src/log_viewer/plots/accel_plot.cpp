// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_flight_log_gui/log_viewer/plots/accel_plot.hpp"

#include <QGridLayout>

#include <tobas_ros2_tools/time.hpp>

namespace tobas
{
namespace gui
{
namespace log
{
AccelPlotWidget::AccelPlotWidget()
  : cur_curves_{ "Current Linear Accel X [m/s²]",    "Current Linear Accel Y [m/s²]",
                 "Current Linear Accel Z [m/s²]",    "Current Angular Accel X [rad/s²]",
                 "Current Angular Accel Y [rad/s²]", "Current Angular Accel Z [rad/s²]" }
  , tar_curves_{ "Target Linear Accel X [m/s²]",    "Target Linear Accel Y [m/s²]",    "Target Linear Accel Z [m/s²]",
                 "Target Angular Accel X [rad/s²]", "Target Angular Accel Y [rad/s²]", "Target Angular Accel Z [rad/s²]" }
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

void AccelPlotWidget::clear()
{
  for (size_t i = 0; i < kNumAxes; ++i) {
    cur_curves_[i].clear();
    tar_curves_[i].clear();
    plots_[i]->replot();
  }
}

void AccelPlotWidget::setTimeScale(double t_start, double t_stop)
{
  for (auto& plot : plots_) {
    plot->setAxisScale(QwtPlot::xBottom, t_start, t_stop);
  }
}

void AccelPlotWidget::setData(
  const QVector<tobas_msgs::msg::OdometryWithCovarianceStamped>& odom_msgs,
  const QVector<tobas_msgs::msg::OdometryStamped>& setpoint_msgs)
{
  updateCurrentSamples(odom_msgs);
  updateTargetSamples(setpoint_msgs);

  for (auto& plot : plots_) {
    plot->replot();
  }
}

void AccelPlotWidget::updateCurrentSamples(const QVector<tobas_msgs::msg::OdometryWithCovarianceStamped>& odom_msgs)
{
  QVector<double> t_data;
  std::array<QVector<double>, kNumAxes> val_data;

  for (const auto& odom : odom_msgs) {
    t_data.push_back(ros2::seconds(odom.header.stamp));

    const auto& lin_acc = odom.odom.odom.accel.linear;
    val_data[0].push_back(lin_acc.x);
    val_data[1].push_back(lin_acc.y);
    val_data[2].push_back(lin_acc.z);

    const auto& ang_acc = odom.odom.odom.accel.angular;
    val_data[3].push_back(ang_acc.x);
    val_data[4].push_back(ang_acc.y);
    val_data[5].push_back(ang_acc.z);
  }

  for (size_t i = 0; i < kNumAxes; ++i) {
    cur_curves_[i].setSamples(t_data, val_data[i]);
  }
}

void AccelPlotWidget::updateTargetSamples(const QVector<tobas_msgs::msg::OdometryStamped>& setpoint_msgs)
{
  QVector<double> t_data;
  std::array<QVector<double>, kNumAxes> val_data;

  for (const auto& setpoint : setpoint_msgs) {
    t_data.push_back(ros2::seconds(setpoint.header.stamp));

    const auto& lin_acc = setpoint.odom.accel.linear;
    val_data[0].push_back(lin_acc.x);
    val_data[1].push_back(lin_acc.y);
    val_data[2].push_back(lin_acc.z);

    const auto& ang_acc = setpoint.odom.accel.angular;
    val_data[3].push_back(ang_acc.x);
    val_data[4].push_back(ang_acc.y);
    val_data[5].push_back(ang_acc.z);
  }

  for (size_t i = 0; i < kNumAxes; ++i) {
    tar_curves_[i].setSamples(t_data, val_data[i]);
  }
}
}  // namespace log
}  // namespace gui
}  // namespace tobas
