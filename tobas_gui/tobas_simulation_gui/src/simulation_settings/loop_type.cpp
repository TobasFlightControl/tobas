#include <QVBoxLayout>

#include <tobas_qt_tools/widgets/label.hpp>

#include "tobas_simulation_gui/simulation_settings/loop_type.hpp"
#include "tobas_simulation_gui/constants.hpp"

namespace gui
{
namespace sim
{
LoopTypeWidget::LoopTypeWidget()
{
  ckb_group_ = new QButtonGroup(this);
  ckb_group_->setExclusive(true);

  // XXX: 選択肢の文が長すぎると500pxに収まらなくなる
  sitl_ckb_ = new QCheckBox("SITL (Simulation in the Loop)");
  hitl_ckb_ = new QCheckBox("HITL (Hardware in the Loop)");

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

loop_type_t LoopTypeWidget::loopType() const
{
  const auto checked_button = ckb_group_->checkedButton();

  if (checked_button == sitl_ckb_) {
    return SITL;
  }
  else if (checked_button == hitl_ckb_) {
    return HITL;
  }
  else {
    throw std::runtime_error("The checked button does not match any of the defined buttons.");
  }
}
}  // namespace sim
}  // namespace gui
