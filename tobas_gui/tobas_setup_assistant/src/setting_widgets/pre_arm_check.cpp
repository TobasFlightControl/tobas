#include <tobas_yaml_tools/convert/qstring.hpp>

#include "tobas_setup_assistant/setting_tabs/pre_arm_check.hpp"

namespace gui
{
namespace sa
{
PreArmCheckWidget::PreArmCheckWidget()
{
  node_connection_ = new QCheckBox("Check node connection");
  battery_voltage_ = new QCheckBox("Check battery voltage");
  cpu_temperature_ = new QCheckBox("Check CPU temperature");
  rotor_communication_ = new QCheckBox("Check rotor communication");
  attitude_level_ = new QCheckBox("Check attitude level");
  position_stability_ = new QCheckBox("Check position stability");
  position_accuracy_ = new QCheckBox("Check position accuracy");
  velocity_accuracy_ = new QCheckBox("Check velocity accuracy");
  attitude_accuracy_ = new QCheckBox("Check attitude accuracy");
  heading_accuracy_ = new QCheckBox("Check heading accuracy");

  node_connection_->setChecked(true);
  battery_voltage_->setChecked(true);
  cpu_temperature_->setChecked(true);
  rotor_communication_->setChecked(true);
  attitude_level_->setChecked(true);
  position_stability_->setChecked(true);
  position_accuracy_->setChecked(true);
  velocity_accuracy_->setChecked(true);
  attitude_accuracy_->setChecked(true);
  heading_accuracy_->setChecked(true);

  addWidget(node_connection_);
  addWidget(battery_voltage_);
  addWidget(cpu_temperature_);
  addWidget(rotor_communication_);
  addWidget(attitude_level_);
  addWidget(position_stability_);
  addWidget(position_accuracy_);
  addWidget(velocity_accuracy_);
  addWidget(attitude_accuracy_);
  addWidget(heading_accuracy_);
  addStretch();
}

const char* PreArmCheckWidget::name() const
{
  return "PreArm Check";
}

const char* PreArmCheckWidget::title() const
{
  return "Configure Pre-Arm Checks";
}

const char* PreArmCheckWidget::description() const
{
  return "";  // TODO
}

void PreArmCheckWidget::onOpened()
{
}

void PreArmCheckWidget::updateInternalDataStructures()
{
}

bool PreArmCheckWidget::isValid()
{
  return true;
}

YAML::Node PreArmCheckWidget::dump()
{
  YAML::Node node(YAML::NodeType::Map);

  node[node_connection_->text()] = node_connection_->isChecked();
  node[battery_voltage_->text()] = battery_voltage_->isChecked();
  node[cpu_temperature_->text()] = cpu_temperature_->isChecked();
  node[rotor_communication_->text()] = rotor_communication_->isChecked();
  node[attitude_level_->text()] = attitude_level_->isChecked();
  node[position_stability_->text()] = position_stability_->isChecked();
  node[position_accuracy_->text()] = position_accuracy_->isChecked();
  node[velocity_accuracy_->text()] = velocity_accuracy_->isChecked();
  node[attitude_accuracy_->text()] = attitude_accuracy_->isChecked();
  node[heading_accuracy_->text()] = heading_accuracy_->isChecked();

  return node;
}

void PreArmCheckWidget::load(const YAML::Node& node)
{
  node_connection_->setChecked(node[node_connection_->text()].as<bool>());
  battery_voltage_->setChecked(node[battery_voltage_->text()].as<bool>());
  cpu_temperature_->setChecked(node[cpu_temperature_->text()].as<bool>());
  rotor_communication_->setChecked(node[rotor_communication_->text()].as<bool>());
  attitude_level_->setChecked(node[attitude_level_->text()].as<bool>());
  position_stability_->setChecked(node[position_stability_->text()].as<bool>());
  position_accuracy_->setChecked(node[position_accuracy_->text()].as<bool>());
  velocity_accuracy_->setChecked(node[velocity_accuracy_->text()].as<bool>());
  attitude_accuracy_->setChecked(node[attitude_accuracy_->text()].as<bool>());
  heading_accuracy_->setChecked(node[heading_accuracy_->text()].as<bool>());
}

bool PreArmCheckWidget::checkNodeConnection() const
{
  return node_connection_->isChecked();
}

bool PreArmCheckWidget::checkBatteryVoltage() const
{
  return battery_voltage_->isChecked();
}

bool PreArmCheckWidget::checkCPUTemperature() const
{
  return cpu_temperature_->isChecked();
}

bool PreArmCheckWidget::checkRotorCommunication() const
{
  return rotor_communication_->isChecked();
}

bool PreArmCheckWidget::checkAttitudeLevel() const
{
  return attitude_level_->isChecked();
}

bool PreArmCheckWidget::checkPositionStability() const
{
  return position_stability_->isChecked();
}

bool PreArmCheckWidget::checkPositionAccuracy() const
{
  return position_accuracy_->isChecked();
}

bool PreArmCheckWidget::checkVelocityAccuracy() const
{
  return velocity_accuracy_->isChecked();
}

bool PreArmCheckWidget::checkAttitudeAccuracy() const
{
  return attitude_accuracy_->isChecked();
}

bool PreArmCheckWidget::checkHeadingAccuracy() const
{
  return heading_accuracy_->isChecked();
}
}  // namespace sa
}  // namespace gui
