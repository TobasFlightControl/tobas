// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_flight_log_gui/log_viewer/plots/cpu_plot.hpp"

#include <QVBoxLayout>

#include <tobas_ros2_tools/time.hpp>

namespace tobas
{
namespace gui
{
namespace log
{
CpuPlotWidget::CpuPlotWidget()
  : freq_curve_("Frequency [GHz]"), temp_curve_("Temperature [degC]"), load_curve_("Load [%]")
{
  const auto rows = new QVBoxLayout();
  setLayout(rows);

  freq_plot_ = new QwtPlot2();
  freq_plot_->setAxisNoLabel(QwtPlot::xBottom);
  freq_plot_->setAxisScale(QwtPlot::yLeft, 0.0, 3.0);
  freq_curve_.setPen(Qt::black, kLineWidth);
  freq_curve_.attach(freq_plot_);
  rows->addWidget(freq_plot_, 1);

  temp_plot_ = new QwtPlot2();
  temp_plot_->setAxisNoLabel(QwtPlot::xBottom);
  temp_plot_->setAxisScale(QwtPlot2::yLeft, 0.0, 100.0);
  temp_curve_.setPen(Qt::black, kLineWidth);
  temp_curve_.attach(temp_plot_);
  rows->addWidget(temp_plot_, 1);

  load_plot_ = new QwtPlot2();
  load_plot_->setAxisNoLabel(QwtPlot::xBottom);
  load_plot_->setAxisScale(QwtPlot::yLeft, 0.0, 100.0);
  load_curve_.setPen(Qt::black, kLineWidth);
  load_curve_.attach(load_plot_);
  rows->addWidget(load_plot_, 1);
}

void CpuPlotWidget::clear()
{
  freq_curve_.clear();
  freq_plot_->replot();

  temp_curve_.clear();
  temp_plot_->replot();

  load_curve_.clear();
  load_plot_->replot();
}

void CpuPlotWidget::setTimeScale(double t_start, double t_stop)
{
  freq_plot_->setAxisScale(QwtPlot::xBottom, t_start, t_stop);
  temp_plot_->setAxisScale(QwtPlot::xBottom, t_start, t_stop);
  load_plot_->setAxisScale(QwtPlot::xBottom, t_start, t_stop);
}

void CpuPlotWidget::setData(const QVector<tobas_msgs::msg::Cpu>& msgs)
{
  QVector<double> t_data;
  QVector<double> freq_data;
  QVector<double> temp_data;
  QVector<double> load_data;

  for (const auto& msg : msgs) {
    t_data.push_back(ros2::seconds(msg.header.stamp));

    freq_data.push_back(static_cast<double>(msg.frequency) * 1e-9);
    temp_data.push_back(msg.temperature);
    load_data.push_back(msg.load * 100.0);
  }

  freq_curve_.setSamples(t_data, freq_data);
  freq_plot_->replot();

  temp_curve_.setSamples(t_data, temp_data);
  temp_plot_->replot();

  load_curve_.setSamples(t_data, load_data);
  load_plot_->replot();
}
}  // namespace log
}  // namespace gui
}  // namespace tobas
