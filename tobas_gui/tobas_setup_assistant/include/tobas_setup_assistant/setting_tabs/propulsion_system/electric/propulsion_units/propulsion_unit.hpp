// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QPushButton>

#include <tobas_qt_tools/widgets/tab_widget.hpp>

#include "./aerodynamics/aerodynamics.hpp"
#include "./esc.hpp"
#include "./motor.hpp"
#include "./propeller.hpp"

namespace tobas::gui::sa::propulsion::electric
{
class PropulsionUnitWidget : public QWidget
{
  Q_OBJECT

Q_SIGNALS:
  void copyToAllButtonClicked();

public:
  explicit PropulsionUnitWidget();

  void setToDefaults();
  bool isValid();
  void copyFrom(const PropulsionUnitWidget* _src);

  YAML::Node dump() const;
  void load(const YAML::Node& _node);

  const ESCWidget* esc() const;
  const MotorWidget* motor() const;
  const PropellerWidget* propeller() const;
  const AerodynamicsWidget* aerodynamics() const;

private:
  qt::TabWidget* tabs_;

  QPushButton* copy_to_all_btn_;

  ESCWidget* esc_;
  MotorWidget* motor_;
  PropellerWidget* propeller_;
  AerodynamicsWidget* aerodynamics_;
};
}  // namespace tobas::gui::sa::propulsion::electric
