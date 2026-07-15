// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_flight_log_gui/log_viewer/plots/rotor_speed_plot.hpp"

#include <algorithm>
#include <cmath>
#include <ranges>

#include <qwt/qwt_scale_engine.h>

#include <tobas_qt_tools/util.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_std_tools/unit_conversions.hpp>

namespace tobas
{
namespace gui
{
namespace log
{
RotorSpeedPlotWidget::RotorSpeedPlotWidget()
{
  grid_ = new qt::GridLayout();
  setLayout(grid_);
}

void RotorSpeedPlotWidget::clear()
{
  // The containers and layout reference the same widgets, so clear the containers first.
  plots_.clear();
  cur_speed_curves_.clear();
  tar_speed_curves_.clear();
  grid_->clear();

  num_rotors_ = 0;
  name2idx_.clear();
}

void RotorSpeedPlotWidget::setTimeScale(double t_start, double t_stop)
{
  for (auto& plot : plots_) {
    plot->setAxisScale(QwtPlot::xBottom, t_start, t_stop);
  }
}

void RotorSpeedPlotWidget::setData(
  const QVector<tobas_msgs::msg::RotorStateArray>& cur_msgs,
  const QVector<tobas_msgs::msg::RotorSpeedArray>& tar_msgs)
{
  if (cur_msgs.size() > 0) {
    const auto& first_msg = cur_msgs.first();
    if (first_msg.states.size() != num_rotors_) {
      if (!updateInternalDataStructures(first_msg)) {
        return;
      }
    }
  }

  const auto [min_cur_speed, max_cur_speed] = updateCurrentSpeedSamples(cur_msgs);
  const auto [min_tar_speed, max_tar_speed] = updateTargetSpeedSamples(tar_msgs);
  const auto min_speed = std::min(min_cur_speed, min_tar_speed);
  const auto max_speed = std::max(max_cur_speed, max_tar_speed);
  if (min_speed <= max_speed) {
    updateVerticalScale(min_speed, max_speed);
  }

  for (auto& plot : plots_) {
    plot->replot();
  }
}

bool RotorSpeedPlotWidget::updateInternalDataStructures(const tobas_msgs::msg::RotorStateArray& msg)
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

    // Arrange widgets in an N-row, 2-column grid.
    const auto row = idx / 2;
    const auto col = idx % 2;
    grid_->addWidget(plot, row, col, 1, 1);
    grid_->setRowStretch(row, 1);
    grid_->setColumnStretch(col, 1);

    qwt::QwtPlotCurveWrapper cur_speed_curve("Current RPM (" + QString::fromStdString(elem.link_name) + ")");
    cur_speed_curve.setPen(kCurrentValueColor, kLineWidth);
    cur_speed_curve.attach(plot);

    qwt::QwtPlotCurveWrapper tar_speed_curve("Target RPM (" + QString::fromStdString(elem.link_name) + ")");
    tar_speed_curve.setPen(kTargetValueColor, kLineWidth);
    tar_speed_curve.attach(plot);

    plots_.push_back(plot);
    cur_speed_curves_.push_back(cur_speed_curve);
    tar_speed_curves_.push_back(tar_speed_curve);
  }

  return true;
}

std::pair<double, double>
RotorSpeedPlotWidget::updateCurrentSpeedSamples(const QVector<tobas_msgs::msg::RotorStateArray>& msgs)
{
  QVector<QVector<double>> t_data(num_rotors_);
  QVector<QVector<double>> speed_data(num_rotors_);
  double min_speed = std::numeric_limits<double>::max();
  double max_speed = 0.0;

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

      if (elem.status == tobas_msgs::msg::RotorState::COMMUNICATION_FAILURE) {
        continue;
      }

      const auto& idx = name2idx_.at(elem.link_name);
      const auto speed = st::rps2rpm(elem.speed);

      t_data[idx].push_back(ros2::seconds(msg.header.stamp));
      speed_data[idx].push_back(speed);

      if (std::isfinite(speed)) {
        min_speed = std::min(min_speed, speed);
        max_speed = std::max(max_speed, speed);
      }
    }
  }

  for (size_t i = 0; i < num_rotors_; ++i) {
    cur_speed_curves_[i].setSamples(t_data[i], speed_data[i]);
  }

  return { min_speed, max_speed };
}

std::pair<double, double>
RotorSpeedPlotWidget::updateTargetSpeedSamples(const QVector<tobas_msgs::msg::RotorSpeedArray>& msgs)
{
  QVector<QVector<double>> t_data(num_rotors_);
  QVector<QVector<double>> speed_data(num_rotors_);
  double min_speed = std::numeric_limits<double>::max();
  double max_speed = 0.0;

  for (const auto& msg : msgs) {
    if (msg.speeds.size() != num_rotors_) {
      qWarning() << "The number of rotors mismatch.";
      continue;
    }

    for (const auto& elem : msg.speeds) {
      if (!name2idx_.contains(elem.link_name)) {
        qWarning() << "Rotor" << QString::fromStdString(elem.link_name) << "is not registered.";
        continue;
      }

      const auto& idx = name2idx_[elem.link_name];
      const auto speed = st::rps2rpm(elem.speed);

      t_data[idx].push_back(ros2::seconds(msg.header.stamp));
      speed_data[idx].push_back(speed);

      if (std::isfinite(speed)) {
        min_speed = std::min(min_speed, speed);
        max_speed = std::max(max_speed, speed);
      }
    }
  }

  for (size_t i = 0; i < num_rotors_; ++i) {
    tar_speed_curves_[i].setSamples(t_data[i], speed_data[i]);
  }

  return { min_speed, max_speed };
}

void RotorSpeedPlotWidget::updateVerticalScale(const double min_speed, const double max_speed)
{
  double scale_min = min_speed;
  double scale_max = std::max(max_speed, 1000.0);
  double scale_step = 0.0;
  QwtLinearScaleEngine().autoScale(kMaxVerticalScaleSteps, scale_min, scale_max, scale_step);

  for (auto& plot : plots_) {
    plot->setAxisScale(QwtPlot::yLeft, scale_min, scale_max, scale_step);
  }
}
}  // namespace log
}  // namespace gui
}  // namespace tobas
