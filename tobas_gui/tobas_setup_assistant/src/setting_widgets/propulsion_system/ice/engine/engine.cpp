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
  ignoreWheelEvent();
  setTabSize(kTabWidth, kTabHeight);

  dynamics_ = new EngineDynamicsWidget();
  response_ = new EngineResponseWidget();
  hw_iface_ = new EngineHardwareIfaceWidget();

  addTab(dynamics_, kDynamicsLabel);
  addTab(response_, kResponseLabel);
  addTab(hw_iface_, kHardwareIfaceLabel);
}

bool EngineWidget::isValid()
{
  if (!dynamics_->isValid())
    return false;

  if (!response_->isValid())
    return false;

  if (!hw_iface_->isValid())
    return false;

  return true;
}

YAML::Node EngineWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[kDynamicsLabel] = dynamics_->dump();
  node[kResponseLabel] = response_->dump();
  node[kHardwareIfaceLabel] = hw_iface_->dump();

  return node;
}

void EngineWidget::load(const YAML::Node& node)
{
  dynamics_->load(node[kDynamicsLabel]);
  response_->load(node[kResponseLabel]);
  hw_iface_->load(node[kHardwareIfaceLabel]);
}

const EngineDynamicsWidget* EngineWidget::dynamics() const
{
  return dynamics_;
}

const EngineResponseWidget* EngineWidget::response() const
{
  return response_;
}

const EngineHardwareIfaceWidget* EngineWidget::hardwareIface() const
{
  return hw_iface_;
}
}  // namespace ice
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
