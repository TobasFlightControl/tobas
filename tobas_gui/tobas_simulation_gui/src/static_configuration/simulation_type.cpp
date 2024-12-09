#include <QVBoxLayout>

#include <tobas_qt_tools/widgets/label.hpp>

#include "tobas_simulation_gui/static_configuration/simulation_type.hpp"
#include "tobas_simulation_gui/constants.hpp"

namespace gui
{
namespace sim
{
SimulationTypeWidget::SimulationTypeWidget()
{
  ckb_group_ = new QButtonGroup(this);
  ckb_group_->setExclusive(true);

  sitl_ckb_ = new QCheckBox("SITL (Simulation in the Loop): The core software will run on the PC.");
  hitl_ckb_ = new QCheckBox("HITL (Hardware in the Loop): The core software will run on the FC.");

  ckb_group_->addButton(sitl_ckb_);
  ckb_group_->addButton(hitl_ckb_);

  // Default
  sitl_ckb_->setChecked(true);

  // Layout
  const auto rows = new QVBoxLayout();
  rows->addWidget(new qt::Label("Simulation Type", kLabelPSize, QFont::Bold));
  rows->addWidget(sitl_ckb_);
  rows->addWidget(hitl_ckb_);

  setLayout(rows);
}

sim_type_t SimulationTypeWidget::simulationType() const
{
  const auto checked_button = ckb_group_->checkedButton();

  if (checked_button == sitl_ckb_)
    return SITL;
  else if (checked_button == hitl_ckb_)
    return HITL;
  else
    throw std::runtime_error("The checked button does not match any of the defined buttons.");
}
}  // namespace sim
}  // namespace gui
