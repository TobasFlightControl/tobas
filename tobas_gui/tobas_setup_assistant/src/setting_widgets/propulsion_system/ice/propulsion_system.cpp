#include "tobas_setup_assistant/setting_tabs/propulsion_system/ice/propulsion_system.hpp"

namespace gui
{
namespace sa
{
namespace propulsion
{
namespace ice
{
PropulsionSystemWidget::PropulsionSystemWidget(rclcpp::Node::SharedPtr node, const RobotInfo& robot, Signals& _signals)
{
  setTabSize(kTabWidth, kTabHeight);

  engine = new EngineWidget();
  units = new PropulsionUnitsWidget(node, robot, _signals);

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
  if (!engine->isValid())
    return false;

  if (!units->isValid())
    return false;

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

bool PropulsionSystemWidget::isTiltRotor(int) const
{
  return false;
}

QString PropulsionSystemWidget::tiltJointName(int) const
{
  return "";
}
}  // namespace ice
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
