// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/setting_tabs/propulsion_system/electric/propulsion_units/esc.hpp"

#include <tobas_qt_tools/cast.hpp>
#include <tobas_qt_tools/layouts/form_layout.hpp>

namespace tobas::gui::sa::propulsion::electric
{
namespace
{
constexpr char kMaxContinuousCurrentLabel[] = "Maximum Continuous Current";
}  // namespace

ESCWidget::ESCWidget()
{
  const auto form = new qt::FormLayout();
  setLayout(form);  // Do not pass `this` to the layout constructor.

  max_cont_current_ = new qt::SpinBox();
  max_cont_current_->setToolTip(
    "Continuous current rating of the ESC in amperes. "
    "Use the continuous rating rather than a short-duration burst rating.");
  max_cont_current_->setMinimum(1);
  max_cont_current_->setSuffix(" A");
  form->addRow(kMaxContinuousCurrentLabel, max_cont_current_);
  form->labelForField(max_cont_current_)->setToolTip(max_cont_current_->toolTip());
}

const char* ESCWidget::name() const
{
  return "ESC";
}

void ESCWidget::setToDefaults()
{
  max_cont_current_->setValue(35);
}

bool ESCWidget::isValid()
{
  return true;
}

void ESCWidget::copyFrom(const BaseSelectedLinkSettingWidget* _src)
{
  const auto derived = qt::qConstPointerCast<ESCWidget>(_src);
  max_cont_current_->setValue(derived->max_cont_current_->value());
}

YAML::Node ESCWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[kMaxContinuousCurrentLabel] = max_cont_current_->value();

  return node;
}

void ESCWidget::load(const YAML::Node& _node)
{
  max_cont_current_->setValue(_node[kMaxContinuousCurrentLabel].as<int>());
}

double ESCWidget::maxContinuousCurrent() const
{
  return max_cont_current_->value();
}
}  // namespace tobas::gui::sa::propulsion::electric
