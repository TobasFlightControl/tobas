// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/setting_tabs/propulsion_system/ice/propulsion_units/transmission.hpp"

#include <utility>

#include <QFormLayout>

#include <tobas_qt_tools/cast.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_yaml_tools/convert/qstring.hpp>

namespace tobas::gui::sa::propulsion::ice
{
namespace
{
constexpr char kGearRatioLabel[] = "Gear Ratio";
}  // namespace

TransmissionWidget::TransmissionWidget()
{
  const auto form = new QFormLayout();
  setLayout(form);  // Do not pass `this` to the layout constructor.

  gear_ratio_ = new RatioWidget();
  gear_ratio_->setToolTip(
    "Engine rotational speed : propeller rotational speed. "
    "For a 2:1 reduction, enter 2 on the engine side and 1 on the propeller side. "
    "The engine value must be at least the propeller value.");
  gear_ratio_->setMinimum(1);
  gear_ratio_->setLeftText("Engine");
  gear_ratio_->setRightText("Propeller");
  form->addRow(kGearRatioLabel, gear_ratio_);
  form->labelForField(gear_ratio_)->setToolTip(gear_ratio_->toolTip());
}

const char* TransmissionWidget::name() const
{
  return "Transmission";
}

void TransmissionWidget::setToDefaults()
{
  gear_ratio_->setValue({ 60 * 33 * 23 * 39, 30 * 44 * 11 * 34 });
}

bool TransmissionWidget::isValid()
{
  const auto [engine_gear, propeller_gear] = gear_ratio_->value();
  if (engine_gear < propeller_gear) {
    qt::qWarnBox(
      this, "The engine's rotational speed must be equal to or greater than the propeller's rotational speed.");
    return false;
  }

  return true;
}

void TransmissionWidget::copyFrom(const BaseSelectedLinkSettingWidget* _src)
{
  const auto derived = qt::qConstPointerCast<TransmissionWidget>(_src);

  gear_ratio_->setValue(derived->gear_ratio_->value());
}

YAML::Node TransmissionWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[kGearRatioLabel] = gear_ratio_->value();

  return node;
}

void TransmissionWidget::load(const YAML::Node& _node)
{
  gear_ratio_->setValue(_node[kGearRatioLabel].as<std::pair<int, int>>());
}

double TransmissionWidget::gearRatio() const
{
  const auto [engine_gear, propeller_gear] = gear_ratio_->value();
  return static_cast<double>(engine_gear) / static_cast<double>(propeller_gear);
}
}  // namespace tobas::gui::sa::propulsion::ice
