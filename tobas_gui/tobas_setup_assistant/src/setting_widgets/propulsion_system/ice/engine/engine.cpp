#include "tobas_setup_assistant/setting_tabs/propulsion_system/ice/engine/engine.hpp"

namespace gui
{
namespace sa
{
namespace propulsion
{
namespace ice
{
EngineWidget::EngineWidget()
{
  setTabSize(kTabWidth, kTabHeight);

  response_ = new EngineResponseWidget();
  dynamics_ = new EngineDynamicsWidget();

  addTab(response_, kResponseLabel);
  addTab(dynamics_, kDynamicsLabel);
}

bool EngineWidget::isValid()
{
  if (!response_->isValid())
    return false;

  if (!dynamics_->isValid())
    return false;

  return true;
}

YAML::Node EngineWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[kResponseLabel] = response_->dump();
  node[kDynamicsLabel] = dynamics_->dump();

  return node;
}

void EngineWidget::load(const YAML::Node& node)
{
  response_->load(node[kResponseLabel]);
  dynamics_->load(node[kDynamicsLabel]);
}

const EngineResponseWidget* EngineWidget::response() const
{
  return response_;
}

const EngineDynamicsWidget* EngineWidget::dynamics() const
{
  return dynamics_;
}
}  // namespace ice
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
