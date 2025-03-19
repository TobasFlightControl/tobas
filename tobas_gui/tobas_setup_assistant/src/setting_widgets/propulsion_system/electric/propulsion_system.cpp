#include "tobas_setup_assistant/setting_tabs/propulsion_system/electric/propulsion_system.hpp"

namespace gui
{
namespace sa
{
namespace propulsion
{
namespace electric
{
PropulsionSystemWidget::PropulsionSystemWidget(rclcpp::Node::SharedPtr node, const RobotInfo& robot)
{
  setTabSize(kTabWidth, kTabHeight);

  battery = new BatteryWidget();
  units = new PropulsionUnitsWidget(node, robot);

  addTab(battery, kBatteryTitle);
  addTab(units, kPropulsionUnitTitle);

  connect(units, &PropulsionUnitsWidget::linkAdded, [this](const QString& link_name) { Q_EMIT linkAdded(link_name); });
  connect(
    units, &PropulsionUnitsWidget::linkRemoved, [this](const QString& link_name) { Q_EMIT linkRemoved(link_name); });
  connect(
    units, &PropulsionUnitsWidget::isTiltStateChanged,
    [this](const QString& link_name, bool is_tilt) { Q_EMIT isTiltStateChanged(link_name, is_tilt); });
  connect(
    units, &PropulsionUnitsWidget::tiltJointNameChanged,
    [this](const QString& link_name, const QString& tilt_joint_name)
    { Q_EMIT tiltJointNameChanged(link_name, tilt_joint_name); });
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
