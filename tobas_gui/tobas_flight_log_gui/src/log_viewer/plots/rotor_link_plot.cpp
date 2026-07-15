// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_flight_log_gui/log_viewer/plots/rotor_link_plot.hpp"

#include <algorithm>
#include <ranges>

#include <tobas_qt_tools/util.hpp>
#include <tobas_ros2_tools/time.hpp>

namespace tobas
{
namespace gui
{
namespace log
{
RotorLinkPlotWidget::RotorLinkPlotWidget()
{
  grid_ = new qt::GridLayout();
  setLayout(grid_);
}

void RotorLinkPlotWidget::clear()
{
  // The containers and layout reference the same widgets, so clear the containers first.
  plots_.clear();
  curves_.clear();
  grid_->clear();

  num_rotors_ = 0;
  name2idx_.clear();
}

void RotorLinkPlotWidget::setTimeScale(double t_start, double t_stop)
{
  for (auto& plot : plots_) {
    plot->setAxisScale(QwtPlot::xBottom, t_start, t_stop);
  }
}

void RotorLinkPlotWidget::setData(const QVector<tobas_msgs::msg::RotorStateArray>& msgs)
{
  if (msgs.size() > 0) {
    const auto& first_msg = msgs.first();
    if (first_msg.states.size() != num_rotors_) {
      if (!updateInternalDataStructures(first_msg)) {
        return;
      }
    }
  }

  QVector<QVector<double>> t_data(num_rotors_);
  QVector<QVector<double>> value_data(num_rotors_);

  for (const auto& msg : msgs) {
    if (msg.states.size() != num_rotors_) {
      qWarning() << "The number of rotors mismatch.";
      continue;
    }

    for (const auto& elem : msg.states) {
      if (!name2idx_.contains(elem.link_name)) {
        qWarning() << "Rotor" << QString::fromStdString(elem.link_name) << "is not registered.";
        continue;
      }

      const auto& idx = name2idx_.at(elem.link_name);

      t_data[idx].push_back(ros2::seconds(msg.header.stamp));

      value_data[idx].push_back(static_cast<int>(elem.status == tobas_msgs::msg::RotorState::COMMUNICATION_FAILURE));
    }
  }

  for (size_t i = 0; i < num_rotors_; ++i) {
    curves_[i].setSamples(t_data[i], value_data[i]);
  }

  for (auto& plot : plots_) {
    plot->replot();
  }
}

bool RotorLinkPlotWidget::updateInternalDataStructures(const tobas_msgs::msg::RotorStateArray& msg)
{
  clear();

  for (const auto& [idx, elem] : std::views::enumerate(msg.states)) {
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
    plot->setupIndexedLabelPlot({ "OK", "ERR" });

    // Arrange widgets in an N-row, 2-column grid.
    const auto row = idx / 2;
    const auto col = idx % 2;
    grid_->addWidget(plot, row, col, 1, 1);
    grid_->setRowStretch(row, 1);
    grid_->setColumnStretch(col, 1);

    qwt::QwtPlotCurveWrapper curve("Communication State (" + QString::fromStdString(elem.link_name) + ")");
    curve.setStyleSteps();
    curve.setPen(Qt::black, kLineWidth);
    curve.attach(plot);

    plots_.push_back(plot);
    curves_.push_back(curve);
  }

  return true;
}
}  // namespace log
}  // namespace gui
}  // namespace tobas
