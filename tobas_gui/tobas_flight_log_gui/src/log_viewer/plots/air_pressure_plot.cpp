// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_flight_log_gui/log_viewer/plots/air_pressure_plot.hpp"

#include <QVBoxLayout>

#include <tobas_ros2_tools/time.hpp>

namespace tobas
{
namespace gui
{
namespace log
{
AirPressurePlotWidget::AirPressurePlotWidget() : curve_("Pressure [hPa]")
{
  const auto rows = new QVBoxLayout();
  setLayout(rows);

  plot_ = new QwtPlot2();
  plot_->setAxisNoLabel(QwtPlot::xBottom);
  rows->addWidget(plot_, 1);

  curve_.setPen(Qt::black, kLineWidth);
  curve_.attach(plot_);
}

void AirPressurePlotWidget::clear()
{
  curve_.clear();
  plot_->replot();
}

void AirPressurePlotWidget::setTimeScale(double t_start, double t_stop)
{
  plot_->setAxisScale(QwtPlot::xBottom, t_start, t_stop);
}

void AirPressurePlotWidget::setData(const QVector<tobas_msgs::msg::FluidPressure>& msgs)
{
  QVector<double> t_data;
  QVector<double> val_data;
  VerticalScaleRange range;

  for (const auto& msg : msgs) {
    t_data.push_back(ros2::seconds(msg.header.stamp));
    val_data.push_back(msg.pressure * 1e-2);
    range.include(val_data.back());
  }

  curve_.setSamples(t_data, val_data);
  setCenteredVerticalScale(*plot_, range, kMinPressureScale);
  plot_->replot();
}
}  // namespace log
}  // namespace gui
}  // namespace tobas
