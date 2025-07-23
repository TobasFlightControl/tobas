#include "tobas_setup_assistant/setting_tabs/propulsion_system/ice/propulsion_system.hpp"

namespace gui
{
namespace sa
{
namespace propulsion
{
namespace ice
{
PropulsionSystemWidget::PropulsionSystemWidget(const uadf::Model& uadf)
{
  engine = new EngineWidget();
  units = new PropulsionUnitsWidget(uadf);

  addTab(engine, kEngineTitle);
  addTab(units, kPropulsionUnitTitle);
}

const char* PropulsionSystemWidget::name() const
{
  return "ICE Propulsion System";
}

void PropulsionSystemWidget::updateInternalDataStructures()
{
  units->updateInternalDataStructures();
}

bool PropulsionSystemWidget::isValid()
{
  if (!engine->isValid()) {
    return false;
  }

  if (!units->isValid()) {
    return false;
  }

  return true;
}

YAML::Node PropulsionSystemWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[kEngineTitle] = engine->dump();
  node[kPropulsionUnitTitle] = units->dump();

  return node;
}

void PropulsionSystemWidget::load(const YAML::Node& node)
{
  engine->load(node[kEngineTitle]);
  units->load(node[kPropulsionUnitTitle]);
}

tobas::PropulsionSystem PropulsionSystemWidget::type() const
{
  return tobas::PropulsionSystem::kIce;
}

int PropulsionSystemWidget::numUnits() const
{
  return units->count();
}

QString PropulsionSystemWidget::linkName(int index) const
{
  return units->linkName(index);
}
}  // namespace ice
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
