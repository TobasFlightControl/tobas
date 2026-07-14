// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_flight_log_gui/log_viewer/plots/mag_plot.hpp"

#include <QVBoxLayout>

#include <tobas_ros2_tools/time.hpp>

namespace tobas
{
namespace gui
{
namespace log
{
MagPlotWidget::MagPlotWidget() : curves_{ "Mag X [-]", "Mag Y [-]", "Mag Z [-]" }
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

void MagPlotWidget::clear()
{
  for (size_t i = 0; i < kNumAxes; ++i) {
    curves_[i].clear();
    plots_[i]->replot();
  }
}

void MagPlotWidget::setTimeScale(double t_start, double t_stop)
{
  for (size_t i = 0; i < kNumAxes; ++i) {
    plots_[i]->setAxisScale(QwtPlot::xBottom, t_start, t_stop);
  }
}

void MagPlotWidget::setData(const QVector<tobas_msgs::msg::MagneticField>& msgs)
{
  QVector<double> t_data;
  std::array<QVector<double>, kNumAxes> mag_data;
  VerticalScaleRange range;

  for (const auto& msg : msgs) {
    t_data.push_back(ros2::seconds(msg.header.stamp));

    mag_data[0].push_back(msg.mag.x);
    mag_data[1].push_back(msg.mag.y);
    mag_data[2].push_back(msg.mag.z);

    range.include(msg.mag.x);
    range.include(msg.mag.y);
    range.include(msg.mag.z);
  }

  for (size_t i = 0; i < kNumAxes; ++i) {
    curves_[i].setSamples(t_data, mag_data[i]);
  }

  setSharedVerticalScale(plots_.begin(), plots_.end(), range);

  for (auto& plot : plots_) {
    plot->replot();
  }
}
}  // namespace log
}  // namespace gui
}  // namespace tobas
