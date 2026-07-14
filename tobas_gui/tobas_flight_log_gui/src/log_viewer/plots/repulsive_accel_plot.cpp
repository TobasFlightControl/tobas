// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_flight_log_gui/log_viewer/plots/repulsive_accel_plot.hpp"

#include <QVBoxLayout>

#include <tobas_ros2_tools/time.hpp>

namespace tobas
{
namespace gui
{
namespace log
{
RepulsiveAccelPlotWidget::RepulsiveAccelPlotWidget() : curves_{ "Accel X [m/s²]", "Accel Y [m/s²]", "Accel Z [m/s²]" }
{
  const auto rows = new QVBoxLayout();
  setLayout(rows);

  for (size_t i = 0; i < kNumAxes; ++i) {
    plots_[i] = new QwtPlot2();
    plots_[i]->setAxisNoLabel(QwtPlot::xBottom);
    curves_[i].setPen(kColorXYZ[i], kLineWidth);
    curves_[i].attach(plots_[i]);
    rows->addWidget(plots_[i], 1);
  }
}

void RepulsiveAccelPlotWidget::clear()
{
  for (size_t i = 0; i < kNumAxes; ++i) {
    curves_[i].clear();
    plots_[i]->replot();
  }
}

void RepulsiveAccelPlotWidget::setTimeScale(double t_start, double t_stop)
{
  for (size_t i = 0; i < kNumAxes; ++i) {
    plots_[i]->setAxisScale(QwtPlot::xBottom, t_start, t_stop);
  }
}

void RepulsiveAccelPlotWidget::setData(const QVector<tobas_msgs::msg::RepulsiveAcceleration>& msgs)
{
  QVector<double> t_data;
  std::array<QVector<double>, kNumAxes> acc_data;
  VerticalScaleRange range;

  for (const auto& msg : msgs) {
    t_data.push_back(ros2::seconds(msg.header.stamp));

    acc_data[0].push_back(msg.accel.x);
    acc_data[1].push_back(msg.accel.y);
    acc_data[2].push_back(msg.accel.z);

    range.include(msg.accel.x);
    range.include(msg.accel.y);
    range.include(msg.accel.z);
  }

  for (size_t i = 0; i < kNumAxes; ++i) {
    curves_[i].setSamples(t_data, acc_data[i]);
  }

  setSharedZeroCenteredVerticalScale(plots_, range);

  for (auto& plot : plots_) {
    plot->replot();
  }
}
}  // namespace log
}  // namespace gui
}  // namespace tobas
