// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_flight_log_gui/log_viewer/plots/dist_force_plot.hpp"

#include <QGridLayout>

#include <tobas_ros2_tools/time.hpp>

namespace tobas
{
namespace gui
{
namespace log
{
DisturbanceForcePlotWidget::DisturbanceForcePlotWidget()
  : curves_{ "Force X [N]", "Force Y [N]", "Force Z [N]", "Torque X [Nm]", "Torque Y [Nm]", "Torque Z [Nm]" }
{
  const auto grid = new QGridLayout();
  setLayout(grid);

  for (size_t i = 0; i < kNumAxes; ++i) {
    plots_[i] = new QwtPlot2();
    plots_[i]->setAxisNoLabel(QwtPlot::xBottom);
    grid->addWidget(plots_[i], i % 3, i / 3, 1, 1);

    curves_[i].setPen(kColorXYZ[i % 3], kLineWidth);
    curves_[i].attach(plots_[i]);
  }
}

void DisturbanceForcePlotWidget::clear()
{
  for (size_t i = 0; i < kNumAxes; ++i) {
    curves_[i].clear();
    plots_[i]->replot();
  }
}

void DisturbanceForcePlotWidget::setTimeScale(double t_start, double t_stop)
{
  for (auto& plot : plots_) {
    plot->setAxisScale(QwtPlot::xBottom, t_start, t_stop);
  }
}

void DisturbanceForcePlotWidget::setData(const QVector<tobas_kdl_msgs::msg::WrenchStamped>& msgs)
{
  QVector<double> t_data;
  std::array<QVector<double>, kNumAxes> val_data;

  for (const auto& msg : msgs) {
    t_data.push_back(ros2::seconds(msg.header.stamp));

    const auto& force = msg.wrench.force;
    val_data[0].push_back(force.x);
    val_data[1].push_back(force.y);
    val_data[2].push_back(force.z);

    const auto& torque = msg.wrench.torque;
    val_data[3].push_back(torque.x);
    val_data[4].push_back(torque.y);
    val_data[5].push_back(torque.z);
  }

  for (size_t i = 0; i < kNumAxes; ++i) {
    curves_[i].setSamples(t_data, val_data[i]);
    plots_[i]->replot();
  }
}
}  // namespace log
}  // namespace gui
}  // namespace tobas
