#include "tobas_setup_assistant/setting_tabs/propulsion_system/electric/propulsion_system.hpp"

namespace gui
{
namespace sa
{
namespace propulsion
{
namespace electric
{
PropulsionSystemWidget::PropulsionSystemWidget(rclcpp::Node::SharedPtr node, const uadf::Model& uadf)
{
  battery = new BatteryWidget();
  units = new PropulsionUnitsWidget(node, uadf);

  addTab(battery, kBatteryTitle);
  addTab(units, kPropulsionUnitTitle);
}

const char* PropulsionSystemWidget::name() const
{
  return "Electric Propulsion System";
}

void PropulsionSystemWidget::updateInternalDataStructures()
{
  units->updateInternalDataStructures();
}

bool PropulsionSystemWidget::isValid()
{
  if (!battery->isValid()) {
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

  node[kBatteryTitle] = battery->dump();
  node[kPropulsionUnitTitle] = units->dump();

  return node;
}

void PropulsionSystemWidget::load(const YAML::Node& node)
{
  battery->load(node[kBatteryTitle]);
  units->load(node[kPropulsionUnitTitle]);
}

tobas::PropulsionSystem PropulsionSystemWidget::type() const
{
  return tobas::PropulsionSystem::kElectric;
}

int PropulsionSystemWidget::numUnits() const
{
  return units->count();
}

QString PropulsionSystemWidget::linkName(int index) const
{
  return units->linkName(index);
}
}  // namespace electric
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
