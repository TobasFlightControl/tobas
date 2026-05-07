// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_flight_log_gui/log_viewer/plots/pwm_plot.hpp"

#include <ranges>

#include <QGridLayout>

#include <tobas_ros2_tools/time.hpp>

namespace tobas
{
namespace gui
{
namespace log
{
PwmPlotWidget::PwmPlotWidget()
{
  const auto grid = new QGridLayout();
  setLayout(grid);

  for (int i = 0; i < kPwmChannels; ++i) {
    const auto plot = new QwtPlot2();
    plot->setAxisNoLabel(QwtPlot::xBottom);
    plot->setAxisScale(QwtPlot::yLeft, 0., 2500.);

    // ウィジェットをN行2列の格子状に配置
    const auto row = i / 2;
    const auto col = i % 2;
    grid->addWidget(plot, row, col, 1, 1);
    grid->setRowStretch(row, 1);
    grid->setColumnStretch(col, 1);

    qwt::QwtPlotCurveWrapper curve("PWM CH" + QString::number(i) + " [us]");
    curve.setPen(Qt::black, kLineWidth);
    curve.attach(plot);

    plots_.append(plot);
    curves_.append(curve);
  }
}

void PwmPlotWidget::clear()
{
  for (const auto& [plot, curve] : std::views::zip(plots_, curves_)) {
    curve.clear();
    plot->replot();
  }
}

void PwmPlotWidget::setTimeScale(double t_start, double t_stop)
{
  for (const auto& plot : plots_) {
    plot->setAxisScale(QwtPlot::xBottom, t_start, t_stop);
  }
}

void PwmPlotWidget::setData(const QVector<tobas_msgs::msg::PwmArray>& msgs)
{
  QVector<QVector<double>> times(kPwmChannels);
  QVector<QVector<double>> values(kPwmChannels);

  for (const auto& msg : msgs) {
    for (const auto& elem : msg.pwms) {
      if (elem.channel >= kPwmChannels) {
        qWarning() << "PWM channel" << (int)elem.channel << "is out of range.";
        continue;
      }
      times[elem.channel].append(ros2::seconds(msg.header.stamp));
      values[elem.channel].append(elem.period);
    }
  }

  for (const auto& [plot, curve, time, value] : std::views::zip(plots_, curves_, times, values)) {
    curve.setSamples(time, value);
    plot->replot();
  }
}
}  // namespace log
}  // namespace gui
}  // namespace tobas
