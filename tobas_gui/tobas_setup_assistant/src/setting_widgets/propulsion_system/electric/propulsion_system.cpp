#include "tobas_setup_assistant/setting_tabs/propulsion_system/electric/propulsion_system.hpp"

namespace gui
{
namespace sa
{
namespace propulsion
{
namespace electric
{
PropulsionSystemWidget::PropulsionSystemWidget(rclcpp::Node::SharedPtr node, const RobotInfo& robot, Signals& _signals)
{
  battery = new BatteryWidget();
  units = new PropulsionUnitsWidget(node, robot, _signals);

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
  if (!battery->isValid())
    return false;

  if (!units->isValid())
    return false;

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

tobas::propulsion_system_t PropulsionSystemWidget::type() const
{
  return tobas::propulsion_system_t::ELECTRIC;
}

int PropulsionSystemWidget::numUnits() const
{
  return units->selected()->count();
}

QString PropulsionSystemWidget::linkName(int index) const
{
  return units->selected()->linkName(index);
}

bool PropulsionSystemWidget::isTiltRotor(int index) const
{
  return units->selected()->widget(index)->general()->isTiltRotor();
}

QString PropulsionSystemWidget::tiltJointName(int index) const
{
  return units->selected()->widget(index)->general()->tiltJointName();
}
}  // namespace electric
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
