#include "tobas_setup_assistant/setting_tabs/propulsion_system/electric/battery/other.hpp"

#include <tobas_qt_tools/message.hpp>

namespace gui
{
namespace sa
{
namespace propulsion
{
namespace electric
{
BatteryWidget_Other::BatteryWidget_Other()
{
  const auto rows = new QVBoxLayout();
  setLayout(rows);

  nominal_voltage_ = new ParamGetterWidget_DoubleSpinBox("Nominal Voltage", "Nominal voltage of the battery.");
  nominal_voltage_->setDecimals(1);
  nominal_voltage_->setMinimum(0.1);
  nominal_voltage_->setValue(14.8);
  nominal_voltage_->setSuffix(" V");
  rows->addWidget(nominal_voltage_);

  max_voltage_ = new ParamGetterWidget_DoubleSpinBox("Maximum Voltage", "Maximum voltage of the battery.");
  max_voltage_->setDecimals(1);
  max_voltage_->setMinimum(0.1);
  max_voltage_->setValue(16.8);
  max_voltage_->setSuffix(" V");
  rows->addWidget(max_voltage_);

  sag_voltage_ = new ParamGetterWidget_DoubleSpinBox(
    "Voltage Threshold", "Voltage at which the discharge characteristics change abruptly.");
  sag_voltage_->setDecimals(1);
  sag_voltage_->setMinimum(0.1);
  sag_voltage_->setValue(13.6);
  sag_voltage_->setSuffix(" V");
  rows->addWidget(sag_voltage_);

  max_current_ = new ParamGetterWidget_DoubleSpinBox("Maximum Current", "Maximum current of the battery.");
  max_current_->setDecimals(1);
  max_current_->setMinimum(0.1);
  max_current_->setValue(200.);
  max_current_->setSuffix(" A");
  rows->addWidget(max_current_);

  capacity_ = new ParamGetterWidget_SpinBox(
    "Current Capacity", "The amount of electric charge that can be drawn from the battery.");
  capacity_->setMinimum(1);
  capacity_->setValue(5000);
  capacity_->setSuffix(" mAh");
  rows->addWidget(capacity_);

  registance_ = new ParamGetterWidget_SpinBox("Internal Registance", "Internal resistance value per cell.");
  registance_->setMinimum(0);
  registance_->setValue(12);
  registance_->setSuffix(" mΩ");
  rows->addWidget(registance_);

  rows->addStretch();
}

const char* BatteryWidget_Other::name() const
{
  return "Other Battery";
}

bool BatteryWidget_Other::isValid()
{
  if (max_voltage_->getValue() <= sag_voltage_->getValue()) {
    qt::qErrorBox(this, "Maximum voltage must be greater than voltage threshold.");
    return false;
  }

  return true;
}

YAML::Node BatteryWidget_Other::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[nominal_voltage_->name()] = nominal_voltage_->getValue();
  node[max_voltage_->name()] = max_voltage_->getValue();
  node[sag_voltage_->name()] = sag_voltage_->getValue();
  node[max_current_->name()] = max_current_->getValue();
  node[capacity_->name()] = capacity_->getValue();
  node[registance_->name()] = registance_->getValue();

  return node;
}

void BatteryWidget_Other::load(const YAML::Node& node)
{
  nominal_voltage_->setValue(node[nominal_voltage_->name()].as<double>());
  max_voltage_->setValue(node[max_voltage_->name()].as<double>());
  sag_voltage_->setValue(node[sag_voltage_->name()].as<double>());
  max_current_->setValue(node[max_current_->name()].as<double>());
  capacity_->setValue(node[capacity_->name()].as<int>());
  registance_->setValue(node[registance_->name()].as<int>());
}

double BatteryWidget_Other::nominalVoltage()
{
  return nominal_voltage_->getValue();
}

double BatteryWidget_Other::maxVoltage()
{
  return max_voltage_->getValue();
}

double BatteryWidget_Other::sagVoltage()
{
  return sag_voltage_->getValue();
}

double BatteryWidget_Other::maxCurrent()
{
  return max_current_->getValue();
}

double BatteryWidget_Other::capacity()
{
  return capacity_->getValue() * 3600 * 1e-3;
}

double BatteryWidget_Other::internalRegistance()
{
  return registance_->getValue() * 1e-3;
}
}  // namespace electric
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
