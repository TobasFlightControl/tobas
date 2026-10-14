// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_qt_tools/layouts/grid_layout.hpp>
#include <tobas_qwt_wrapper/qwt_plot_curve.hpp>

#include <tobas_msgs/msg/rotor_speed_array.hpp>
#include <tobas_msgs/msg/rotor_state_array.hpp>

#include "./utilities/utilities.hpp"

namespace tobas::gui::log
{
class RotorSpeedPlotWidget : public BasePlotWidget
{
  Q_OBJECT

public:
  explicit RotorSpeedPlotWidget();

  void clear() override;
  void setTimeScale(double _t_start, double _t_stop) override;

  void setData(
    const QVector<tobas_msgs::msg::RotorStateArray>& _cur_msgs,
    const QVector<tobas_msgs::msg::RotorSpeedArray>& _tar_msgs);

private:
  QVector<QwtPlot2*> plots_;

  QVector<qwt::QwtPlotCurveWrapper> cur_speed_curves_;
  QVector<qwt::QwtPlotCurveWrapper> tar_speed_curves_;

  qt::GridLayout* grid_;

  size_t num_rotors_;                                 ///< The number of rotors
  std::unordered_map<std::string, size_t> name2idx_;  ///< Link Name -> Index

  bool updateInternalDataStructures(const tobas_msgs::msg::RotorStateArray& _msg);

  std::pair<double, double> updateCurrentSpeedSamples(const QVector<tobas_msgs::msg::RotorStateArray>& _msgs);
  std::pair<double, double> updateTargetSpeedSamples(const QVector<tobas_msgs::msg::RotorSpeedArray>& _msgs);
  void updateVerticalScale(double _min_speed, double _max_speed);
};
}  // namespace tobas::gui::log
