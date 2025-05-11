#include "tobas_setup_assistant/setting_tabs/propulsion_system/ice/engine/dynamics/manual.hpp"

namespace gui
{
namespace sa
{
namespace propulsion
{
namespace ice
{
EngineDynamicsWidget_Manual::EngineDynamicsWidget_Manual()
{
  engine_const_ = new ParamGetterWidget_DoublePair("Engine Constant", "");  // TODO
  engine_const_->setDecimals(12);
  engine_const_->setMinimum(1e-12);
  engine_const_->setValue({ 0.0012411595525231814, 0.01850453250677594 });
  addWidget(engine_const_);
}

const char* EngineDynamicsWidget_Manual::name() const
{
  return "Set Manually";
}

const char* EngineDynamicsWidget_Manual::description() const
{
  return "Directly set the engine dynamics constants.";
}

bool EngineDynamicsWidget_Manual::isValid()
{
  return true;
}

YAML::Node EngineDynamicsWidget_Manual::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[engine_const_->name()] = engine_const_->getValue();

  return node;
}

void EngineDynamicsWidget_Manual::load(const YAML::Node& node)
{
  engine_const_->setValue(node[engine_const_->name()].as<std::pair<double, double>>());
}

std::pair<double, double> EngineDynamicsWidget_Manual::engineConstant() const
{
  return engine_const_->getValue();
}
}  // namespace ice
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
