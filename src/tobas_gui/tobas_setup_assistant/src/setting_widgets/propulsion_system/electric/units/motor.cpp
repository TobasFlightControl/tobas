// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/setting_tabs/propulsion_system/electric/propulsion_units/motor.hpp"

#include <tobas_qt_tools/cast.hpp>
#include <tobas_qt_tools/layouts/form_layout.hpp>
#include <tobas_std_tools/unit_conversions.hpp>
#include <tobas_yaml_tools/convert/qstring.hpp>

namespace tobas::gui::sa::propulsion::electric
{
namespace
{
constexpr char kNumPolesLabel[] = "Number of Poles";
constexpr char kKvLabel[] = "Kv";
constexpr char kResistanceLabel[] = "Internal Resistance";
constexpr char kMinSpeedLabel[] = "Minimum Rotational Speed";
}  // namespace

MotorWidget::MotorWidget()
{
  const auto form = new qt::FormLayout();
  setLayout(form);  // Do not pass `this` to the layout constructor.

  num_poles_ = new qt::SpinBox();
  num_poles_->setToolTip(
    "Total number of magnetic poles on the motor rotor, counting both north and south poles. "
    "Enter the pole count, not the number of pole pairs; for example, 14 poles means 7 pole pairs.");
  num_poles_->setSingleStep(2);
  num_poles_->setMinimum(2);
  form->addRow(kNumPolesLabel, num_poles_);
  form->labelForField(num_poles_)->setToolTip(num_poles_->toolTip());

  kv_ = new qt::SpinBox();
  kv_->setToolTip("No-load motor speed per volt, in rpm/V. Use the motor manufacturer's Kv rating.");
  kv_->setMinimum(1);
  kv_->setSuffix(" rpm/V");
  form->addRow(kKvLabel, kv_);
  form->labelForField(kv_)->setToolTip(kv_->toolTip());

  resistance_ = new qt::SpinBox();
  resistance_->setToolTip("Internal resistance of the motor in milliohms, used by the electrical motor model.");
  resistance_->setMinimum(1);
  resistance_->setSuffix(" mΩ");
  form->addRow(kResistanceLabel, resistance_);
  form->labelForField(resistance_)->setToolTip(resistance_->toolTip());

  min_speed_ = new qt::SpinBox();
  min_speed_->setToolTip("Minimum rotational speed in rpm at which the motor can run continuously and smoothly.");
  min_speed_->setMinimum(0);
  min_speed_->setSuffix(" rpm");
  form->addRow(kMinSpeedLabel, min_speed_);
  form->labelForField(min_speed_)->setToolTip(min_speed_->toolTip());
}

const char* MotorWidget::name() const
{
  return "Motor";
}

void MotorWidget::setToDefaults()
{
  num_poles_->setValue(14);
  kv_->setValue(920);
  resistance_->setValue(250);
  min_speed_->setValue(300);
}

bool MotorWidget::isValid()
{
  return true;
}

void MotorWidget::copyFrom(const BaseSelectedLinkSettingWidget* _src)
{
  const auto derived = qt::qConstPointerCast<MotorWidget>(_src);

  num_poles_->setValue(derived->num_poles_->value());
  kv_->setValue(derived->kv_->value());
  resistance_->setValue(derived->resistance_->value());
  min_speed_->setValue(derived->min_speed_->value());
}

YAML::Node MotorWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[kNumPolesLabel] = num_poles_->value();
  node[kKvLabel] = kv_->value();
  node[kResistanceLabel] = resistance_->value();
  node[kMinSpeedLabel] = min_speed_->value();

  return node;
}

void MotorWidget::load(const YAML::Node& _node)
{
  num_poles_->setValue(_node[kNumPolesLabel].as<int>());
  kv_->setValue(_node[kKvLabel].as<int>());
  resistance_->setValue(_node[kResistanceLabel].as<int>());
  min_speed_->setValue(_node[kMinSpeedLabel].as<int>());
}

int MotorWidget::numPoles() const
{
  return num_poles_->value();
}

double MotorWidget::kv() const
{
  return st::rpm2rps(kv_->value());
}

double MotorWidget::internalResistance() const
{
  return resistance_->value() * 1e-3;
}

double MotorWidget::minimumSpeed() const
{
  return st::rpm2rps(min_speed_->value());
}
}  // namespace tobas::gui::sa::propulsion::electric
