// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_flight_log_gui/log_viewer/plots/latency_plot.hpp"

#include <QGridLayout>

#include <tobas_ros2_tools/time.hpp>

namespace tobas
{
namespace gui
{
namespace log
{
namespace
{
constexpr int kSamplingTimeMin = 500;     // [us]
constexpr int kSamplingTimeMax = 2000;    // [us]
constexpr int kControlLatencyMin = 0;     // [us]
constexpr int kControlLatencyMax = 1500;  // [us]
constexpr int kBinWidth = 10;             // [us]
}  // namespace

LatencyPlotWidget::LatencyPlotWidget()
  : sampling_time_curve_("IMU Sampling Interval [us]")
  , ctrl_latency_curve_("Control Latency [us]")
  , sampling_time_hist_data_("IMU Sampling Interval Histogram")
  , ctrl_latency_hist_data_("Control Latency Histogram")
{
  sampling_time_plot_ = new QwtPlot2();
  sampling_time_plot_->setAxisNoLabel(QwtPlot::xBottom);
  sampling_time_plot_->setAxisScale(QwtPlot::yLeft, kSamplingTimeMin, kSamplingTimeMax);
  sampling_time_curve_.setPen(Qt::black, kLineWidth);
  sampling_time_curve_.attach(sampling_time_plot_);

  ctrl_latency_plot_ = new QwtPlot2();
  ctrl_latency_plot_->setAxisNoLabel(QwtPlot::xBottom);
  ctrl_latency_plot_->setAxisScale(QwtPlot::yLeft, kControlLatencyMin, kControlLatencyMax);
  ctrl_latency_curve_.setPen(Qt::black, kLineWidth);
  ctrl_latency_curve_.attach(ctrl_latency_plot_);

  sampling_time_hist_plot_ = new QwtPlot2();
  sampling_time_hist_plot_->setAxisScale(QwtPlot::xBottom, kSamplingTimeMin, kSamplingTimeMax);
  sampling_time_hist_plot_->setAxisLabelUnit(QwtPlot::xBottom, "us");
  sampling_time_hist_data_.setPen(Qt::black, kLineWidth);
  sampling_time_hist_data_.setBrush(Qt::gray);
  sampling_time_hist_data_.attach(sampling_time_hist_plot_);

  ctrl_latency_hist_plot_ = new QwtPlot2();
  ctrl_latency_hist_plot_->setAxisScale(QwtPlot::xBottom, kControlLatencyMin, kControlLatencyMax);
  ctrl_latency_hist_plot_->setAxisLabelUnit(QwtPlot::xBottom, "us");
  ctrl_latency_hist_data_.setPen(Qt::black, kLineWidth);
  ctrl_latency_hist_data_.setBrush(Qt::gray);
  ctrl_latency_hist_data_.attach(ctrl_latency_hist_plot_);

  // Layout
  const auto grid = new QGridLayout();
  setLayout(grid);
  grid->addWidget(sampling_time_plot_, 0, 0, 1, 1);
  grid->addWidget(ctrl_latency_plot_, 1, 0, 1, 1);
  grid->addWidget(sampling_time_hist_plot_, 0, 1, 1, 1);
  grid->addWidget(ctrl_latency_hist_plot_, 1, 1, 1, 1);
}

void LatencyPlotWidget::clear()
{
  sampling_time_curve_.clear();
  sampling_time_plot_->replot();

  ctrl_latency_curve_.clear();
  ctrl_latency_plot_->replot();

  sampling_time_hist_data_.clear();
  sampling_time_hist_plot_->replot();

  ctrl_latency_hist_data_.clear();
  ctrl_latency_hist_plot_->replot();
}

void LatencyPlotWidget::setTimeScale(double t_start, double t_stop)
{
  sampling_time_plot_->setAxisScale(QwtPlot::xBottom, t_start, t_stop);
  ctrl_latency_plot_->setAxisScale(QwtPlot::xBottom, t_start, t_stop);
}

void LatencyPlotWidget::setData(
  const QVector<tobas_msgs::msg::Latency>& sampling_time_msgs,
  const QVector<tobas_msgs::msg::Latency>& ctrl_latency_msgs)
{
  setSamplingTimeData(sampling_time_msgs);
  setControlLatencyData(ctrl_latency_msgs);
}

void LatencyPlotWidget::setSamplingTimeData(const QVector<tobas_msgs::msg::Latency>& msgs)
{
  QVector<double> t_data;
  QVector<double> val_data;

  for (const auto& msg : msgs) {
    t_data.push_back(ros2::seconds(msg.header.stamp));
    val_data.push_back(ros2::microseconds(msg.data));
  }

  sampling_time_curve_.setSamples(t_data, val_data);
  sampling_time_plot_->replot();

  sampling_time_hist_data_.setSamples(val_data, kSamplingTimeMin, kSamplingTimeMax, kBinWidth);
  sampling_time_hist_plot_->replot();
}

void LatencyPlotWidget::setControlLatencyData(const QVector<tobas_msgs::msg::Latency>& msgs)
{
  QVector<double> t_data;
  QVector<double> val_data;

  for (const auto& msg : msgs) {
    t_data.push_back(ros2::seconds(msg.header.stamp));
    val_data.push_back(ros2::microseconds(msg.data));
  }

  ctrl_latency_curve_.setSamples(t_data, val_data);
  ctrl_latency_plot_->replot();

  ctrl_latency_hist_data_.setSamples(val_data, kControlLatencyMin, kControlLatencyMax, kBinWidth);
  ctrl_latency_hist_plot_->replot();
}
}  // namespace log
}  // namespace gui
}  // namespace tobas
