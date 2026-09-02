// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/setting_tabs/propulsion_system/ice/engine/engine.hpp"

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
constexpr char kDynamicsLabel[] = "Dynamics";
constexpr char kResponseLabel[] = "Response";
}  // namespace

EngineWidget::EngineWidget()
{
  enableWheelEvent(false);
  setTabSize(135, 45);

  dynamics_ = new EngineDynamicsWidget();
  response_ = new EngineResponseWidget();

  addTab(dynamics_, kDynamicsLabel);
  addTab(response_, kResponseLabel);
}

void EngineWidget::setToDefaults()
{
  dynamics_->setToDefaults();
  response_->setToDefaults();
}

bool EngineWidget::isValid()
{
  if (!dynamics_->isValid()) {
    return false;
  }

  if (!response_->isValid()) {
    return false;
  }

  return true;
}

YAML::Node EngineWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[kDynamicsLabel] = dynamics_->dump();
  node[kResponseLabel] = response_->dump();

  return node;
}

void EngineWidget::load(const YAML::Node& node)
{
  dynamics_->load(node[kDynamicsLabel]);
  response_->load(node[kResponseLabel]);
}

const EngineDynamicsWidget* EngineWidget::dynamics() const
{
  return dynamics_;
}

const EngineResponseWidget* EngineWidget::response() const
{
  return response_;
}
}  // namespace ice
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
}  // namespace tobas
