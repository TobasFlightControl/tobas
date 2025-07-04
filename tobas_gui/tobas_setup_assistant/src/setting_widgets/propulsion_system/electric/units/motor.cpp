#include "tobas_setup_assistant/setting_tabs/propulsion_system/electric/propulsion_units/motor.hpp"

#include <tobas_qt_tools/cast.hpp>
#include <tobas_std_tools/unit_conversions.hpp>
#include <tobas_yaml_tools/convert/qstring.hpp>

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

  num_poles_ = new ParamGetterWidget_SpinBox(
    "Number of Poles",
    "The number of magnetic poles arranged on the rotor inside the motor. "
    "It is the number of pairs of N and S poles of permanent magnets attached to the rotor.");
  num_poles_->setSingleStep(2);
  num_poles_->setMinimum(2);
  num_poles_->setValue(14);
  rows->addWidget(num_poles_);

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

  min_speed_ = new ParamGetterWidget_SpinBox(
    "Minimum Rotation Speed", "The minimum rotational speed at which the motor can spin smoothly.");
  min_speed_->setMinimum(0);
  min_speed_->setValue(300);
  min_speed_->setSuffix(" rpm");
  rows->addWidget(min_speed_);

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
  const auto derived = qt::qConstPointerCast<MotorWidget>(src);

  num_poles_->setValue(derived->num_poles_->getValue());
  kv_->setValue(derived->kv_->getValue());
  resistance_->setValue(derived->resistance_->getValue());
  min_speed_->setValue(derived->min_speed_->getValue());
}

YAML::Node MotorWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[num_poles_->name()] = num_poles_->getValue();
  node[kv_->name()] = kv_->getValue();
  node[resistance_->name()] = resistance_->getValue();
  node[min_speed_->name()] = min_speed_->getValue();

  return node;
}

void MotorWidget::load(const YAML::Node& node)
{
  num_poles_->setValue(node[num_poles_->name()].as<int>());
  kv_->setValue(node[kv_->name()].as<int>());
  resistance_->setValue(node[resistance_->name()].as<int>());
  min_speed_->setValue(node[min_speed_->name()].as<int>());
}

int MotorWidget::numPoles() const
{
  return num_poles_->getValue();
}

double MotorWidget::kv() const
{
  return tobas_std::rpm2rps(kv_->getValue());
}

double MotorWidget::internalResistance() const
{
  return resistance_->getValue() * 1e-3;
}

double MotorWidget::minimumSpeed() const
{
  return tobas_std::rpm2rps(min_speed_->getValue());
}
}  // namespace electric
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
