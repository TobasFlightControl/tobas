#include <tobas_std_tools/unit_conversions.hpp>
#include <tobas_yaml_tools/convert/qstring.hpp>
#include <tobas_drone_core/turning_direction.hpp>

#include "tobas_setup_assistant/setting_tabs/propulsion_system/motor.hpp"

namespace gui
{
namespace setup_assistant
{
MotorWidget::MotorWidget()
{
  const auto rows = new QVBoxLayout(this);

  direction_ = new ParamGetterWidget_ComboBox(
    "Turning Direction", "Motor rotation direction. "
                         "Please choose either CW (Clockwise) or CCW (Counter Clockwise) "
                         "relative to the rotation axis. "
                         "For instance, in rotary-wing aircraft, "
                         "propellers positioned diagonally opposite each other "
                         "typically rotate in the same direction.");
  direction_->setChoices({ tobas::turning_direction::kCWName, tobas::turning_direction::kCCWName });
  rows->addWidget(direction_);

  kv_ =
    new ParamGetterWidget_SpinBox("Kv", "Motor's rotational speed under no load, relative to the supplied voltage.");
  kv_->setMinimum(1);
  kv_->setValue(920);
  kv_->setSuffix(" rpm/V");
  rows->addWidget(kv_);

  resistance_ = new ParamGetterWidget_SpinBox("Internal Registance", "Internal resistance value of the motor.");
  resistance_->setMinimum(1);
  resistance_->setValue(250);
  resistance_->setSuffix(" mΩ");
  rows->addWidget(resistance_);

  num_poles_ = new ParamGetterWidget_SpinBox(
    "Number of Poles", "The number of magnetic poles arranged on the rotor inside the motor. "
                       "It is the number of pairs of N and S poles of permanent magnets attached to the rotor.");
  num_poles_->setSingleStep(2);
  num_poles_->setMinimum(2);
  num_poles_->setValue(14);
  rows->addWidget(num_poles_);

  time_const_up_ = new ParamGetterWidget_SpinBox(
    "Time Constant Up", "Time constant of the motor's response when increasing its rotational speed, "
                        "relative to the command value.");
  time_const_up_->setMinimum(1);
  time_const_up_->setValue(15);
  time_const_up_->setSuffix(" ms");
  rows->addWidget(time_const_up_);

  time_const_down_ = new ParamGetterWidget_SpinBox(
    "Time Constant Down", "Time constant of the motor's response when decreasing its rotational speed, "
                          "relative to the command value.");
  time_const_down_->setMinimum(1);
  time_const_down_->setValue(30);
  time_const_down_->setSuffix(" ms");
  rows->addWidget(time_const_down_);

  rows->addStretch();
}

const char* MotorWidget::name()
{
  return "Motor";
}

bool MotorWidget::isValid()
{
  return true;
}

void MotorWidget::copyFrom(const MotorWidget* src)
{
  direction_->setValue(src->direction_->getValue());
  kv_->setValue(src->kv_->getValue());
  resistance_->setValue(src->resistance_->getValue());
  num_poles_->setValue(src->num_poles_->getValue());
  time_const_up_->setValue(src->time_const_up_->getValue());
  time_const_down_->setValue(src->time_const_down_->getValue());
}

YAML::Node MotorWidget::dump()
{
  YAML::Node node(YAML::NodeType::Map);

  node[direction_->name()] = direction_->getValue();
  node[kv_->name()] = kv_->getValue();
  node[resistance_->name()] = resistance_->getValue();
  node[num_poles_->name()] = num_poles_->getValue();
  node[time_const_up_->name()] = time_const_up_->getValue();
  node[time_const_down_->name()] = time_const_down_->getValue();

  return node;
}

void MotorWidget::load(const YAML::Node& node)
{
  direction_->setValue(node[direction_->name()].as<QString>());
  kv_->setValue(node[kv_->name()].as<int>());
  resistance_->setValue(node[resistance_->name()].as<int>());
  num_poles_->setValue(node[num_poles_->name()].as<int>());
  time_const_up_->setValue(node[time_const_up_->name()].as<int>());
  time_const_down_->setValue(node[time_const_down_->name()].as<int>());
}

std::string MotorWidget::direction() const
{
  return direction_->getValue().toStdString();
}

double MotorWidget::kv() const
{
  return tobas_std::rpm2rps(kv_->getValue());
}

double MotorWidget::internalResistance() const
{
  return resistance_->getValue() * 1e-3;
}

int MotorWidget::numPoles() const
{
  return num_poles_->getValue();
}

double MotorWidget::timeConstUp() const
{
  return time_const_up_->getValue() * 1e-3;
}

double MotorWidget::timeConstDown() const
{
  return time_const_down_->getValue() * 1e-3;
}
}  // namespace setup_assistant
}  // namespace gui
