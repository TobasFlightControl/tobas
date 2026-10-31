// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_flight_log_gui/log_viewer/plots/additional_accel_plot.hpp"

#include <QVBoxLayout>

#include <tobas_ros2_tools/time.hpp>

namespace tobas::gui::log
{
AdditionalAccelPlotWidget::AdditionalAccelPlotWidget() : curves_{ "Accel X [m/s²]", "Accel Y [m/s²]", "Accel Z [m/s²]" }
{
  const auto rows = new QVBoxLayout(this);

  for (size_t i = 0; i < kNumAxes; ++i) {
    plots_[i] = new QwtPlot2();
    plots_[i]->setAxisNoLabel(QwtPlot::xBottom);
    curves_[i].setPen(kColorXYZ[i], kLineWidth);
    curves_[i].attach(plots_[i]);
    rows->addWidget(plots_[i], 1);
  }
}

void AdditionalAccelPlotWidget::clear()
{
  for (size_t i = 0; i < kNumAxes; ++i) {
    curves_[i].clear();
    plots_[i]->replot();
  }
}

void AdditionalAccelPlotWidget::setTimeScale(double _t_start, double _t_stop)
{
  for (size_t i = 0; i < kNumAxes; ++i) {
    plots_[i]->setAxisScale(QwtPlot::xBottom, _t_start, _t_stop);
  }
}

void AdditionalAccelPlotWidget::setData(const QVector<tobas_kdl_msgs::msg::AccelStamped>& _msgs)
{
  QVector<double> t_data;
  std::array<QVector<double>, kNumAxes> acc_data;
  VerticalScaleRange range;

  for (const auto& msg : _msgs) {
    t_data.push_back(ros2::seconds(msg.header.stamp));

    acc_data[0].push_back(msg.accel.linear.x);
    acc_data[1].push_back(msg.accel.linear.y);
    acc_data[2].push_back(msg.accel.linear.z);

    range.include(msg.accel.linear.x);
    range.include(msg.accel.linear.y);
    range.include(msg.accel.linear.z);
  }

  for (size_t i = 0; i < kNumAxes; ++i) {
    curves_[i].setSamples(t_data, acc_data[i]);
  }

  setSharedZeroCenteredVerticalScale(plots_, range);

  for (auto& plot : plots_) {
    plot->replot();
  }
}
}  // namespace tobas::gui::log
