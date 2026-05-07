// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QPushButton>

#include <tobas_qt_tools/widgets/tab_widget.hpp>

#include "./aerodynamics/aerodynamics.hpp"
#include "./esc.hpp"
#include "./motor.hpp"
#include "./propeller.hpp"

namespace tobas
{
namespace gui
{
namespace sa
{
namespace propulsion
{
namespace electric
{
class PropulsionUnitWidget : public QWidget
{
  Q_OBJECT

  static constexpr int kButtonWidth = 125;
  static constexpr int kButtonHeight = 50;
  static constexpr int kTabWidth = 120;
  static constexpr int kTabHeight = 40;

Q_SIGNALS:
  void copyToAllButtonClicked();

public:
  explicit PropulsionUnitWidget(rclcpp::Node::SharedPtr node);

  bool isValid();
  void copyFrom(const PropulsionUnitWidget* src);

  YAML::Node dump() const;
  void load(const YAML::Node& node);

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
}  // namespace electric
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
}  // namespace tobas
