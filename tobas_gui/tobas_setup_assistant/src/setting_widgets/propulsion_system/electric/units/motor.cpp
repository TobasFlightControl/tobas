#include <tobas_std_tools/unit_conversions.hpp>
#include <tobas_yaml_tools/convert/qstring.hpp>

#include "tobas_setup_assistant/setting_tabs/propulsion_system/electric/propulsion_units/motor.hpp"

namespace gui
{
namespace sa
{
namespace propulsion
{
namespace electric
{
MotorWidget::MotorWidget()
{
  const auto rows = new QVBoxLayout();
  setLayout(rows);

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

  rows->addStretch();
}

const char* MotorWidget::name() const
{
  return "Motor";
}

bool MotorWidget::isValid()
{
  return true;
}

void MotorWidget::copyFrom(const BaseSelectedLinkSettingWidget* src)
{
  const auto derived = qobject_cast<const MotorWidget*>(src);

  kv_->setValue(derived->kv_->getValue());
  resistance_->setValue(derived->resistance_->getValue());
  num_poles_->setValue(derived->num_poles_->getValue());
}

YAML::Node MotorWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[kv_->name()] = kv_->getValue();
  node[resistance_->name()] = resistance_->getValue();
  node[num_poles_->name()] = num_poles_->getValue();

  return node;
}

void MotorWidget::load(const YAML::Node& node)
{
  kv_->setValue(node[kv_->name()].as<int>());
  resistance_->setValue(node[resistance_->name()].as<int>());
  num_poles_->setValue(node[num_poles_->name()].as<int>());
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
}  // namespace electric
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
