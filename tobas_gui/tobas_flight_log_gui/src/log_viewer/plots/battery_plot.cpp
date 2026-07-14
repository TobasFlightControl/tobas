// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_flight_log_gui/log_viewer/plots/battery_plot.hpp"

#include <algorithm>

#include <QVBoxLayout>

#include <tobas_ros2_tools/time.hpp>

namespace tobas
{
namespace gui
{
namespace log
{
BatteryPlotWidget::BatteryPlotWidget() : voltage_curve_("Voltage [V]"), current_curve_("Current [A]")
{
  const auto rows = new QVBoxLayout();
  setLayout(rows);

  voltage_plot_ = new QwtPlot2();
  voltage_plot_->setAxisNoLabel(QwtPlot::xBottom);
  voltage_curve_.setPen(Qt::black, kLineWidth);
  voltage_curve_.attach(voltage_plot_);
  rows->addWidget(voltage_plot_, 1);

  current_plot_ = new QwtPlot2();
  current_plot_->setAxisNoLabel(QwtPlot::xBottom);
  current_curve_.setPen(Qt::black, kLineWidth);
  current_curve_.attach(current_plot_);
  rows->addWidget(current_plot_, 1);
}

void BatteryPlotWidget::clear()
{
  voltage_curve_.clear();
  voltage_plot_->replot();

  current_curve_.clear();
  current_plot_->replot();
}

void BatteryPlotWidget::setTimeScale(double t_start, double t_stop)
{
  voltage_plot_->setAxisScale(QwtPlot::xBottom, t_start, t_stop);
  current_plot_->setAxisScale(QwtPlot::xBottom, t_start, t_stop);
}

void BatteryPlotWidget::setData(const QVector<tobas_msgs::msg::Battery>& batt_msgs)
{
  QVector<double> t_data;
  QVector<double> voltage_data;
  QVector<double> current_data;
  VerticalScaleRange voltage_range;
  VerticalScaleRange current_range;

  for (const auto& batt : batt_msgs) {
    t_data.push_back(ros2::seconds(batt.header.stamp));

    voltage_data.push_back(batt.voltage);
    current_data.push_back(batt.current);

    voltage_range.include(voltage_data.back());
    if (current_data.back() > 0.0) {
      current_range.include(current_data.back());
    }
  }

  voltage_curve_.setSamples(t_data, voltage_data);
  setCenteredVerticalScale(*voltage_plot_, voltage_range, kMinVoltageScale);
  voltage_plot_->replot();

  current_curve_.setSamples(t_data, current_data);
  const auto current_scale_min = std::max(current_range.center() - kMinCurrentScale, 0.0);
  current_range.include(current_scale_min);
  current_range.include(current_scale_min + 2 * kMinCurrentScale);
  setVerticalScale(*current_plot_, current_range);
  current_plot_->replot();
}
}  // namespace log
}  // namespace gui
}  // namespace tobas
