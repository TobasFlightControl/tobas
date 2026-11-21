// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/setting_tabs/propulsion_system/electric/propulsion_units/esc.hpp"

#include <tobas_qt_tools/cast.hpp>
#include <tobas_qt_tools/layouts/form_layout.hpp>

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
constexpr char kMaxCurrentKey[] = "Maximum Current";
}  // namespace

ESCWidget::ESCWidget()
{
  const auto form = new qt::FormLayout();
  setLayout(form);

  max_current_ = new qt::SpinBox();
  max_current_->setToolTip("Maximum current that the ESC (Electronic Speed Controller) can safely handle. "
                           "Exceeding this maximum current may lead to overheating or damage to the ESC, "
                           "and in the worst case, it could cause failure or fire.");
  max_current_->setMinimum(1);
  max_current_->setValue(35);
  max_current_->setSuffix(" A");
  form->addRow(kMaxCurrentKey, max_current_);
  form->labelForField(max_current_)->setToolTip(max_current_->toolTip());
}

const char* ESCWidget::name() const
{
  return "ESC";
}

bool ESCWidget::isValid()
{
  return true;
}

void ESCWidget::copyFrom(const BaseSelectedLinkSettingWidget* src)
{
  const auto derived = qt::qConstPointerCast<ESCWidget>(src);
  max_current_->setValue(derived->max_current_->value());
}

YAML::Node ESCWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[kMaxCurrentKey] = max_current_->value();

  return node;
}

void ESCWidget::load(const YAML::Node& node)
{
  max_current_->setValue(node[kMaxCurrentKey].as<int>());
}

double ESCWidget::maxCurrent() const
{
  return max_current_->value();
}
}  // namespace electric
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
}  // namespace tobas
