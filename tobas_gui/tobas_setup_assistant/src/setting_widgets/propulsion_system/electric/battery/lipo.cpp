#include "tobas_setup_assistant/setting_tabs/propulsion_system/electric/battery/lipo.hpp"

namespace gui
{
namespace sa
{
namespace propulsion
{
namespace electric
{
BatteryWidget_LiPo::BatteryWidget_LiPo()
{
  const auto rows = new QVBoxLayout();
  setLayout(rows);

  num_cells_ = new ParamGetterWidget_SpinBox("Number of Cells", "The number of cells in the battery.");
  num_cells_->setMinimum(1);
  num_cells_->setMaximum(100);
  num_cells_->setValue(4);
  rows->addWidget(num_cells_);

  capacity_ = new ParamGetterWidget_SpinBox(
    "Current Capacity", "The amount of electric charge that can be drawn from the battery.");
  capacity_->setMinimum(1);
  capacity_->setValue(5000);
  capacity_->setSuffix(" mAh");
  rows->addWidget(capacity_);

  C_cont_ = new ParamGetterWidget_SpinBox(
    "Continuous Discharge Current Rate", "The maximum continuous discharge current that the battery can provide.");
  C_cont_->setMinimum(1);
  C_cont_->setValue(50);
  C_cont_->setSuffix(" /h");
  rows->addWidget(C_cont_);

  registance_ = new ParamGetterWidget_SpinBox("Internal Registance", "Internal resistance value per cell.");
  registance_->setMinimum(0);
  registance_->setValue(3);
  registance_->setSuffix(" mΩ");
  rows->addWidget(registance_);

  rows->addStretch();
}

const char* BatteryWidget_LiPo::name() const
{
  return "Lithium Polymer Battery (LiPo)";
}

bool BatteryWidget_LiPo::isValid()
{
  return true;
}

YAML::Node BatteryWidget_LiPo::dump()
{
  YAML::Node node(YAML::NodeType::Map);

  node[num_cells_->name()] = num_cells_->getValue();
  node[capacity_->name()] = capacity_->getValue();
  node[C_cont_->name()] = C_cont_->getValue();
  node[registance_->name()] = registance_->getValue();

  return node;
}

void BatteryWidget_LiPo::load(const YAML::Node& node)
{
  num_cells_->setValue(node[num_cells_->name()].as<int>());
  capacity_->setValue(node[capacity_->name()].as<int>());
  C_cont_->setValue(node[C_cont_->name()].as<int>());
  registance_->setValue(node[registance_->name()].as<int>());
}

double BatteryWidget_LiPo::nominalVoltage()
{
  return num_cells_->getValue() * kNominalVoltagePerCell;
}

double BatteryWidget_LiPo::maxVoltage()
{
  return num_cells_->getValue() * kMaxVoltagePerCell;
}

double BatteryWidget_LiPo::sagVoltage()
{
  return num_cells_->getValue() * kSagVoltagePerCell;
}

double BatteryWidget_LiPo::maxCurrent()
{
  return capacity_->getValue() * C_cont_->getValue() * 1e-3;
}

double BatteryWidget_LiPo::capacity()
{
  return capacity_->getValue() * 3600 * 1e-3;
}

double BatteryWidget_LiPo::internalRegistance()
{
  return num_cells_->getValue() * registance_->getValue() * 1e-3;
}
}  // namespace electric
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
