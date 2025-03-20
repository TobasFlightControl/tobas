#include <tobas_qt_tools/cast.hpp>

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
  torque_const_ = new ParamGetterWidget_DoubleSpinBox("Torque Constant", "");  // TODO
  torque_const_->setDecimals(3);
  torque_const_->setMinimum(1e-3);
  torque_const_->setValue(0.1);
  torque_const_->setSuffix(" Nm/(rad/s)");
  addWidget(torque_const_);

  friction_torque_ = new ParamGetterWidget_DoubleSpinBox("Dynamic Friction Torque", "");  // TODO
  friction_torque_->setDecimals(3);
  friction_torque_->setMinimum(1e-3);
  friction_torque_->setValue(1.);
  friction_torque_->setSuffix(" Nm");
  addWidget(friction_torque_);
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

  node[torque_const_->name()] = torque_const_->getValue();
  node[friction_torque_->name()] = friction_torque_->getValue();

  return node;
}

void EngineDynamicsWidget_Manual::load(const YAML::Node& node)
{
  torque_const_->setValue(node[torque_const_->name()].as<double>());
  friction_torque_->setValue(node[friction_torque_->name()].as<double>());
}

double EngineDynamicsWidget_Manual::torqueConstant() const
{
  return torque_const_->getValue();
}

double EngineDynamicsWidget_Manual::dynamicFrictionTorque() const
{
  return friction_torque_->getValue();
}
}  // namespace ice
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
