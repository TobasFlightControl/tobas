// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_flight_log_gui/log_viewer/plots/rc_input_plot.hpp"

#include <QVBoxLayout>

#include <tobas_constants/rc_input.hpp>
#include <tobas_ros2_tools/time.hpp>

namespace tobas
{
namespace gui
{
namespace log
{
RcInputPlotWidget::RcInputPlotWidget()
  : roll_curve_("Roll")
  , pitch_curve_("Pitch")
  , throt_curve_("Throttle")
  , yaw_curve_("Yaw")
  , mode_curve_("Flight Mode")
  , sub_mode_curve_("Sub Mode")
  , enable_curve_("Enable")
  , kill_curve_("Kill")
{
  const auto grid = new QGridLayout();
  setLayout(grid);

  roll_plot_ = new QwtPlot2();
  roll_plot_->setAxisNoLabel(QwtPlot::xBottom);
  roll_plot_->setAxisScale(QwtPlot::yLeft, kRcInputMin, kRcInputMax);
  roll_curve_.setPen(Qt::black, kLineWidth);
  roll_curve_.attach(roll_plot_);
  grid->addWidget(roll_plot_, 0, 0, 1, 1);

  pitch_plot_ = new QwtPlot2();
  pitch_plot_->setAxisNoLabel(QwtPlot::xBottom);
  pitch_plot_->setAxisScale(QwtPlot::yLeft, kRcInputMin, kRcInputMax);
  pitch_curve_.setPen(Qt::black, kLineWidth);
  pitch_curve_.attach(pitch_plot_);
  grid->addWidget(pitch_plot_, 1, 0, 1, 1);

  throt_plot_ = new QwtPlot2();
  throt_plot_->setAxisNoLabel(QwtPlot::xBottom);
  throt_plot_->setAxisScale(QwtPlot::yLeft, kRcInputMin, kRcInputMax);
  throt_curve_.setPen(Qt::black, kLineWidth);
  throt_curve_.attach(throt_plot_);
  grid->addWidget(throt_plot_, 2, 0, 1, 1);

  yaw_plot_ = new QwtPlot2();
  yaw_plot_->setAxisNoLabel(QwtPlot::xBottom);
  yaw_plot_->setAxisScale(QwtPlot::yLeft, kRcInputMin, kRcInputMax);
  yaw_curve_.setPen(Qt::black, kLineWidth);
  yaw_curve_.attach(yaw_plot_);
  grid->addWidget(yaw_plot_, 3, 0, 1, 1);

  mode_plot_ = new QwtPlot2();
  mode_plot_->setAxisNoLabel(QwtPlot::xBottom);
  mode_plot_->setupIndexedLabelPlot({ "Acrobat", "Stabilize", "Loiter" });
  mode_curve_.setPen(Qt::black, kLineWidth);
  mode_curve_.attach(mode_plot_);
  grid->addWidget(mode_plot_, 0, 1, 1, 1);

  sub_mode_plot_ = new QwtPlot2();
  sub_mode_plot_->setAxisNoLabel(QwtPlot::xBottom);
  sub_mode_plot_->setupIndexedLabelPlot({ "OFF", "ON" });
  sub_mode_curve_.setPen(Qt::black, kLineWidth);
  sub_mode_curve_.attach(sub_mode_plot_);
  grid->addWidget(sub_mode_plot_, 1, 1, 1, 1);

  enable_plot_ = new QwtPlot2();
  enable_plot_->setAxisNoLabel(QwtPlot::xBottom);
  enable_plot_->setupIndexedLabelPlot({ "OFF", "ON" });
  enable_curve_.setPen(Qt::black, kLineWidth);
  enable_curve_.attach(enable_plot_);
  grid->addWidget(enable_plot_, 2, 1, 1, 1);

  kill_plot_ = new QwtPlot2();
  kill_plot_->setAxisNoLabel(QwtPlot::xBottom);
  kill_plot_->setupIndexedLabelPlot({ "OFF", "ON" });
  kill_curve_.setPen(Qt::black, kLineWidth);
  kill_curve_.attach(kill_plot_);
  grid->addWidget(kill_plot_, 3, 1, 1, 1);
}

void RcInputPlotWidget::clear()
{
  roll_curve_.clear();
  roll_plot_->replot();

  pitch_curve_.clear();
  pitch_plot_->replot();

  throt_curve_.clear();
  throt_plot_->replot();

  yaw_curve_.clear();
  yaw_plot_->replot();

  mode_curve_.clear();
  mode_plot_->replot();

  sub_mode_curve_.clear();
  sub_mode_plot_->replot();

  enable_curve_.clear();
  enable_plot_->replot();

  kill_curve_.clear();
  kill_plot_->replot();
}

void RcInputPlotWidget::setTimeScale(double t_start, double t_stop)
{
  roll_plot_->setAxisScale(QwtPlot::xBottom, t_start, t_stop);
  pitch_plot_->setAxisScale(QwtPlot::xBottom, t_start, t_stop);
  throt_plot_->setAxisScale(QwtPlot::xBottom, t_start, t_stop);
  yaw_plot_->setAxisScale(QwtPlot::xBottom, t_start, t_stop);
  mode_plot_->setAxisScale(QwtPlot::xBottom, t_start, t_stop);
  sub_mode_plot_->setAxisScale(QwtPlot::xBottom, t_start, t_stop);
  enable_plot_->setAxisScale(QwtPlot::xBottom, t_start, t_stop);
  kill_plot_->setAxisScale(QwtPlot::xBottom, t_start, t_stop);
}

void RcInputPlotWidget::setData(const QVector<tobas_msgs::msg::RCInput>& msgs)
{
  QVector<double> t_data;
  QVector<double> roll_data;
  QVector<double> pitch_data;
  QVector<double> throt_data;
  QVector<double> yaw_data;
  QVector<double> mode_data;
  QVector<double> sub_mode_data;
  QVector<double> enable_data;
  QVector<double> kill_data;

  for (const auto& msg : msgs) {
    if (!msg.ok) {
      continue;
    }

    t_data.push_back(ros2::seconds(msg.header.stamp));

    roll_data.push_back(msg.roll);
    pitch_data.push_back(msg.pitch);
    throt_data.push_back(msg.throttle);
    yaw_data.push_back(msg.yaw);

    mode_data.push_back(msg.mode);
    sub_mode_data.push_back(static_cast<int>(msg.sub_mode));
    enable_data.push_back(static_cast<int>(msg.enable));
    kill_data.push_back(static_cast<int>(msg.kill));
  }

  roll_curve_.setSamples(t_data, roll_data);
  roll_plot_->replot();

  pitch_curve_.setSamples(t_data, pitch_data);
  pitch_plot_->replot();

  throt_curve_.setSamples(t_data, throt_data);
  throt_plot_->replot();

  yaw_curve_.setSamples(t_data, yaw_data);
  yaw_plot_->replot();

  mode_curve_.setSamples(t_data, mode_data);
  mode_plot_->replot();

  sub_mode_curve_.setSamples(t_data, sub_mode_data);
  sub_mode_plot_->replot();

  enable_curve_.setSamples(t_data, enable_data);
  enable_plot_->replot();

  kill_curve_.setSamples(t_data, kill_data);
  kill_plot_->replot();
}
}  // namespace log
}  // namespace gui
}  // namespace tobas
