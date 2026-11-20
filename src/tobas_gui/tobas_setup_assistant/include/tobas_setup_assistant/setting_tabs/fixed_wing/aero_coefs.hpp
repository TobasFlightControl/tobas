// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <yaml-cpp/yaml.h>
#include <QSettings>

#include <tobas_qt_tools/layouts/form_layout.hpp>
#include <tobas_qt_tools/widgets/double_spin_box.hpp>
#include <tobas_qt_tools/widgets/scroll_area.hpp>

namespace tobas::gui::sa::fw
{
class AerodynamicsCoefficientsWidget : public qt::ScrollArea
{
  Q_OBJECT

  using self = AerodynamicsCoefficientsWidget;

public:
  explicit AerodynamicsCoefficientsWidget();

  void updateInternalDataStructures();
  void setToDefaults();
  bool isValid();

  YAML::Node dump() const;
  void load(const YAML::Node& _node);

  double c_lift_0() const;
  double c_lift_alpha() const;
  double c_drag_0() const;
  double c_drag_alpha() const;
  double c_drag_alpha2() const;
  double c_side_beta() const;
  double c_side_p() const;
  double c_side_r() const;
  double c_roll_beta() const;
  double c_roll_p() const;
  double c_roll_r() const;
  double c_pitch_0() const;
  double c_pitch_alpha() const;
  double c_pitch_abs_beta() const;
  double c_pitch_alpha_rate() const;
  double c_pitch_q() const;
  double c_yaw_beta() const;
  double c_yaw_p() const;
  double c_yaw_r() const;

private:
  QSettings settings_store_;

  qt::FormLayout* form_;

  qt::DoubleSpinBox* c_lift_0_;
  qt::DoubleSpinBox* c_lift_alpha_;
  qt::DoubleSpinBox* c_drag_0_;
  qt::DoubleSpinBox* c_drag_alpha_;
  qt::DoubleSpinBox* c_drag_alpha2_;
  qt::DoubleSpinBox* c_side_beta_;
  qt::DoubleSpinBox* c_side_p_;
  qt::DoubleSpinBox* c_side_r_;
  qt::DoubleSpinBox* c_roll_beta_;
  qt::DoubleSpinBox* c_roll_p_;
  qt::DoubleSpinBox* c_roll_r_;
  qt::DoubleSpinBox* c_pitch_0_;
  qt::DoubleSpinBox* c_pitch_alpha_;
  qt::DoubleSpinBox* c_pitch_abs_beta_;
  qt::DoubleSpinBox* c_pitch_alpha_rate_;
  qt::DoubleSpinBox* c_pitch_q_;
  qt::DoubleSpinBox* c_yaw_beta_;
  qt::DoubleSpinBox* c_yaw_p_;
  qt::DoubleSpinBox* c_yaw_r_;

private Q_SLOTS:
  void onLoadButtonClicked();
};
}  // namespace tobas::gui::sa::fw
