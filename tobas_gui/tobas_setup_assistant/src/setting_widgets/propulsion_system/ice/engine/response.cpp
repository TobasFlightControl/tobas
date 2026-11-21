// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/setting_tabs/propulsion_system/ice/engine/response.hpp"

#include <tobas_qt_tools/layouts/form_layout.hpp>
#include <tobas_yaml_tools/convert/qstring.hpp>

namespace tobas
{
namespace gui
{
namespace sa
{
namespace propulsion
{
namespace ice
{
namespace
{
// Keep the existing serialized keys independent of the form labels.
constexpr char kTimeConstUpKey[] = "Time Constant Up";
constexpr char kTimeConstDownKey[] = "Time Constant Down";
}  // namespace

EngineResponseWidget::EngineResponseWidget()
{
  const auto form = new qt::FormLayout();
  setLayout(form);

  time_const_up_ = new qt::SpinBox();  // TODO
  time_const_up_->setMinimum(0);
  time_const_up_->setValue(100);
  time_const_up_->setSuffix(" ms");
  form->addRow(kTimeConstUpKey, time_const_up_);
  form->labelForField(time_const_up_)->setToolTip(time_const_up_->toolTip());

  time_const_down_ = new qt::SpinBox();  // TODO
  time_const_down_->setMinimum(0);
  time_const_down_->setValue(200);
  time_const_down_->setSuffix(" ms");
  form->addRow(kTimeConstDownKey, time_const_down_);
  form->labelForField(time_const_down_)->setToolTip(time_const_down_->toolTip());
}

bool EngineResponseWidget::isValid()
{
  return true;
}

YAML::Node EngineResponseWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[kTimeConstUpKey] = time_const_up_->value();
  node[kTimeConstDownKey] = time_const_down_->value();

  return node;
}

void EngineResponseWidget::load(const YAML::Node& node)
{
  time_const_up_->setValue(node[kTimeConstUpKey].as<int>());
  time_const_down_->setValue(node[kTimeConstDownKey].as<int>());
}

double EngineResponseWidget::timeConstUp() const
{
  return time_const_up_->value() * 1e-3;
}

double EngineResponseWidget::timeConstDown() const
{
  return time_const_down_->value() * 1e-3;
}
}  // namespace ice
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
}  // namespace tobas
