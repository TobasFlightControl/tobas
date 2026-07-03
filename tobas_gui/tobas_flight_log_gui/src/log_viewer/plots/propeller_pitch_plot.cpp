// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_flight_log_gui/log_viewer/plots/propeller_pitch_plot.hpp"

#include <ranges>

#include <tobas_qt_tools/util.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_std_tools/unit_conversions.hpp>

namespace tobas
{
namespace gui
{
namespace log
{
PropellerPitchPlotWidget::PropellerPitchPlotWidget()
{
  grid_ = new qt::GridLayout();
  setLayout(grid_);
}

void PropellerPitchPlotWidget::clear()
{
  // When the same widgets are contained in both the layout and the container, clear the container before the layout.
  plots_.clear();
  curves_.clear();
  grid_->clear();

  num_rotors_ = 0;
  name2idx_.clear();
}

void PropellerPitchPlotWidget::setTimeScale(double t_start, double t_stop)
{
  for (auto& plot : plots_) {
    plot->setAxisScale(QwtPlot::xBottom, t_start, t_stop);
  }
}

void PropellerPitchPlotWidget::setData(const QVector<tobas_msgs::msg::IcePropulsionSystemCommand>& msgs)
{
  if (msgs.empty()) {
    return;
  }

  const auto& first_msg = msgs.first();
  if (first_msg.pitch_angles.size() != num_rotors_) {
    if (!updateInternalDataStructures(first_msg)) {
      return;
    }
  }

  QVector<QVector<double>> t_data(num_rotors_);
  QVector<QVector<double>> pitch_data(num_rotors_);

  for (const auto& msg : msgs) {
    if (msg.pitch_angles.size() != num_rotors_) {
      qWarning() << "The number of VPPs mismatch.";
      continue;
    }

    for (const auto& elem : msg.pitch_angles) {
      if (!name2idx_.contains(elem.link_name)) {
        qWarning() << "VPP" << QString::fromStdString(elem.link_name) << "is not registered.";
        continue;
      }

      const auto& idx = name2idx_.at(elem.link_name);

      t_data[idx].push_back(ros2::seconds(msg.header.stamp));
      pitch_data[idx].push_back(st::rad2deg(elem.angle));
    }
  }

  for (size_t i = 0; i < num_rotors_; ++i) {
    curves_[i].setSamples(t_data[i], pitch_data[i]);
  }

  for (auto& plot : plots_) {
    plot->replot();
  }
}

bool PropellerPitchPlotWidget::updateInternalDataStructures(const tobas_msgs::msg::IcePropulsionSystemCommand& msg)
{
  clear();

  for (const auto& [idx, elem] : std::views::enumerate(msg.pitch_angles)) {
    if (elem.link_name.empty()) {
      qWarning() << "Rotor link name is empty.";
      return false;
    }

    if (!name2idx_.insert({ elem.link_name, idx }).second) {
      qWarning() << "Rotor" << QString::fromStdString(elem.link_name) << "is duplicated.";
      return false;
    }

    ++num_rotors_;

    const auto plot = new QwtPlot2();
    plot->setAxisNoLabel(QwtPlot::xBottom);

    // Arrange widgets in an N-row, 2-column grid.
    const auto row = idx / 2;
    const auto col = idx % 2;
    grid_->addWidget(plot, row, col, 1, 1);
    grid_->setRowStretch(row, 1);
    grid_->setColumnStretch(col, 1);

    qwt::QwtPlotCurveWrapper curve("Pitch Angle [deg] (" + QString::fromStdString(elem.link_name) + ")");
    curve.setPen(kCurrentValueColor, kLineWidth);
    curve.attach(plot);

    plots_.push_back(plot);
    curves_.push_back(curve);
  }

  return true;
}
}  // namespace log
}  // namespace gui
}  // namespace tobas
