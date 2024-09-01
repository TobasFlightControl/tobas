#include "tobas_setup_assistant/setting_tabs/propulsion_system/aerodynamics/manual.hpp"

namespace gui
{
namespace setup_assistant
{
namespace propulsion_system
{
AerodynamicsWidget_Manual::AerodynamicsWidget_Manual()
{
}

const char* AerodynamicsWidget_Manual::name() const
{
  return "Set Manually";
}

const char* AerodynamicsWidget_Manual::description() const
{
  return "Directly set the propeller aerodynamic constants.";
}

void AerodynamicsWidget_Manual::onInit()
{
  motor_const_ = new ParamGetterWidget_DoubleSpinBox(
    "Motor Constant", "Propeller thrust constant. "
                      "If the thrust constant is denoted as c [kg*m/rad^2] and the rotational speed as w [rad/s], "
                      "the thrust force T [N] generated perpendicular to the rotational plane is expressed "
                      "as T = c w^2.");
  motor_const_->setDecimals(12);
  motor_const_->setMinimum(0.);
  motor_const_->setValue(8.54858e-6);
  motor_const_->setSuffix(" kg m/rad^2");
  addWidget(motor_const_);

  moment_const_ = new ParamGetterWidget_DoubleSpinBox(
    "Moment Constant", "Propeller torque reaction coefficient. "
                       "If the torque reaction coefficient is c [m] and the propeller's thrust is T [N], "
                       "the torque generated in the opposite direction to the propeller's rotation, "
                       "in Newton-meters, is N = c T.");
  moment_const_->setDecimals(6);
  moment_const_->setMinimum(0.);
  moment_const_->setValue(0.016);
  moment_const_->setSuffix(" m");
  addWidget(moment_const_);

  rotor_drag_coef_ = new ParamGetterWidget_DoubleSpinBox(
    "Rotor Drag Coefficient", "Propeller drag coefficient. "
                              "If the drag coefficient is c [kg/rad], the motor's rotational speed is w [rad/s], "
                              "and V [m/s] is the magnitude of the atmospheric velocity component "
                              "perpendicular to the rotational axis relative to the aircraft, "
                              "then the magnitude of the air drag force F [N] generated on the propeller is expressed "
                              "as F = c w V.");
  rotor_drag_coef_->setDecimals(9);
  rotor_drag_coef_->setMinimum(0.);
  rotor_drag_coef_->setValue(8.06428e-5);
  rotor_drag_coef_->setSuffix(" kg/rad");
  addWidget(rotor_drag_coef_);
}

bool AerodynamicsWidget_Manual::isValid()
{
  return true;
}

void AerodynamicsWidget_Manual::copyFrom(const AerodynamicsWidget_Base* src)
{
  const auto derived = qobject_cast<const AerodynamicsWidget_Manual*>(src);
  motor_const_->setValue(derived->motor_const_->getValue());
  moment_const_->setValue(derived->moment_const_->getValue());
  rotor_drag_coef_->setValue(derived->rotor_drag_coef_->getValue());
}

YAML::Node AerodynamicsWidget_Manual::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[motor_const_->name()] = motor_const_->getValue();
  node[moment_const_->name()] = moment_const_->getValue();
  node[rotor_drag_coef_->name()] = rotor_drag_coef_->getValue();

  return node;
}

void AerodynamicsWidget_Manual::load(const YAML::Node& node)
{
  motor_const_->setValue(node[motor_const_->name()].as<double>());
  moment_const_->setValue(node[moment_const_->name()].as<double>());
  rotor_drag_coef_->setValue(node[rotor_drag_coef_->name()].as<double>());
}

double AerodynamicsWidget_Manual::motorConst() const
{
  return motor_const_->getValue();
}

double AerodynamicsWidget_Manual::momentConst() const
{
  return moment_const_->getValue();
}

double AerodynamicsWidget_Manual::rotorDragCoef() const
{
  return rotor_drag_coef_->getValue();
}
}  // namespace propulsion_system
}  // namespace setup_assistant
}  // namespace gui
