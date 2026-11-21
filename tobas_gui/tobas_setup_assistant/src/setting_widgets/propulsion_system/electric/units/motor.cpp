// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/setting_tabs/propulsion_system/electric/propulsion_units/motor.hpp"

#include <tobas_qt_tools/cast.hpp>
#include <tobas_qt_tools/layouts/form_layout.hpp>
#include <tobas_std_tools/unit_conversions.hpp>
#include <tobas_yaml_tools/convert/qstring.hpp>

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
namespace
{
// Keep the existing serialized keys independent of the form labels.
constexpr char kNumPolesKey[] = "Number of Poles";
constexpr char kKvKey[] = "Kv";
constexpr char kResistanceKey[] = "Internal Registance";
constexpr char kMinSpeedKey[] = "Minimum Rotation Speed";
}  // namespace

MotorWidget::MotorWidget()
{
  const auto form = new qt::FormLayout();
  setLayout(form);

  num_poles_ = new qt::SpinBox();
  num_poles_->setToolTip("The number of magnetic poles arranged on the rotor inside the motor. "
                         "It is the number of pairs of N and S poles of permanent magnets attached to the rotor.");
  num_poles_->setSingleStep(2);
  num_poles_->setMinimum(2);
  num_poles_->setValue(14);
  form->addRow(kNumPolesKey, num_poles_);
  form->labelForField(num_poles_)->setToolTip(num_poles_->toolTip());

  kv_ = new qt::SpinBox();
  kv_->setToolTip("Motor's rotational speed under no load, relative to the supplied voltage.");
  kv_->setMinimum(1);
  kv_->setValue(920);
  kv_->setSuffix(" rpm/V");
  form->addRow(kKvKey, kv_);
  form->labelForField(kv_)->setToolTip(kv_->toolTip());

  resistance_ = new qt::SpinBox();
  resistance_->setToolTip("Internal resistance value of the motor.");
  resistance_->setMinimum(1);
  resistance_->setValue(250);
  resistance_->setSuffix(" mΩ");
  form->addRow(kResistanceKey, resistance_);
  form->labelForField(resistance_)->setToolTip(resistance_->toolTip());

  min_speed_ = new qt::SpinBox();
  min_speed_->setToolTip("The minimum rotational speed at which the motor can spin smoothly.");
  min_speed_->setMinimum(0);
  min_speed_->setValue(300);
  min_speed_->setSuffix(" rpm");
  form->addRow(kMinSpeedKey, min_speed_);
  form->labelForField(min_speed_)->setToolTip(min_speed_->toolTip());
}

const char* MotorWidget::name() const
{
  return "Motor";
}

bool MotorWidget::isValid()
{
  return true;
}

void MotorWidget::copyFrom(const BaseSelectedLinkSettingWidget* src)
{
  const auto derived = qt::qConstPointerCast<MotorWidget>(src);

  num_poles_->setValue(derived->num_poles_->value());
  kv_->setValue(derived->kv_->value());
  resistance_->setValue(derived->resistance_->value());
  min_speed_->setValue(derived->min_speed_->value());
}

YAML::Node MotorWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[kNumPolesKey] = num_poles_->value();
  node[kKvKey] = kv_->value();
  node[kResistanceKey] = resistance_->value();
  node[kMinSpeedKey] = min_speed_->value();

  return node;
}

void MotorWidget::load(const YAML::Node& node)
{
  num_poles_->setValue(node[kNumPolesKey].as<int>());
  kv_->setValue(node[kKvKey].as<int>());
  resistance_->setValue(node[kResistanceKey].as<int>());
  min_speed_->setValue(node[kMinSpeedKey].as<int>());
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
}  // namespace electric
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
}  // namespace tobas
