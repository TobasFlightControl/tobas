// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/setting_tabs/propulsion_system/ice/engine/response.hpp"

#include <tobas_qt_tools/layouts/form_layout.hpp>
#include <tobas_yaml_tools/convert/qstring.hpp>

namespace tobas::gui::sa::propulsion::ice
{
namespace
{
constexpr char kTimeConstUpLabel[] = "Speed Increase Time Constant";
constexpr char kTimeConstDownLabel[] = "Speed Decrease Time Constant";
}  // namespace

EngineResponseWidget::EngineResponseWidget()
{
  const auto form = new qt::FormLayout(this);

  time_const_up_ = new qt::SpinBox();
  time_const_up_->setToolTip(
    "First-order time constant in milliseconds for increasing engine speed in simulation. "
    "A larger value produces a slower response.");
  time_const_up_->setMinimum(0);
  time_const_up_->setSuffix(" ms");
  form->addRow(kTimeConstUpLabel, time_const_up_);
  form->labelForField(time_const_up_)->setToolTip(time_const_up_->toolTip());

  time_const_down_ = new qt::SpinBox();
  time_const_down_->setToolTip(
    "First-order time constant in milliseconds for decreasing engine speed in simulation. "
    "A larger value produces a slower response.");
  time_const_down_->setMinimum(0);
  time_const_down_->setSuffix(" ms");
  form->addRow(kTimeConstDownLabel, time_const_down_);
  form->labelForField(time_const_down_)->setToolTip(time_const_down_->toolTip());
}

void EngineResponseWidget::setToDefaults()
{
  time_const_up_->setValue(100);
  time_const_down_->setValue(200);
}

bool EngineResponseWidget::isValid()
{
  return true;
}

YAML::Node EngineResponseWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[kTimeConstUpLabel] = time_const_up_->value();
  node[kTimeConstDownLabel] = time_const_down_->value();

  return node;
}

void EngineResponseWidget::load(const YAML::Node& _node)
{
  time_const_up_->setValue(_node[kTimeConstUpLabel].as<int>());
  time_const_down_->setValue(_node[kTimeConstDownLabel].as<int>());
}

double EngineResponseWidget::timeConstUp() const
{
  return time_const_up_->value() * 1e-3;
}

double EngineResponseWidget::timeConstDown() const
{
  return time_const_down_->value() * 1e-3;
}
}  // namespace tobas::gui::sa::propulsion::ice
