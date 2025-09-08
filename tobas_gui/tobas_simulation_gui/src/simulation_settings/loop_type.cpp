#include "tobas_simulation_gui/simulation_settings/loop_type.hpp"

#include <QVBoxLayout>

#include <tobas_gui_common/constants.hpp>
#include <tobas_qt_tools/widgets/label.hpp>

namespace gui
{
namespace sim
{
LoopTypeWidget::LoopTypeWidget()
{
  btn_group_ = new QButtonGroup(this);
  btn_group_->setExclusive(true);

  // 選択肢の文が長すぎると500pxに収まらなくなる
  sitl_btn_ = new QRadioButton("SITL (Simulation in the Loop)");
  hitl_btn_ = new QRadioButton("HITL (Hardware in the Loop)");

  hitl_btn_->setEnabled(false);  // TODO: HITLできるようにして有効化

  btn_group_->addButton(sitl_btn_);
  btn_group_->addButton(hitl_btn_);

  // Default
  sitl_btn_->setChecked(true);

  // Layout
  const auto rows = new QVBoxLayout();
  rows->addWidget(new qt::Label("Simulation Type", common::kLabelPSize, QFont::Bold));
  rows->addWidget(sitl_btn_);
  rows->addWidget(hitl_btn_);

  setLayout(rows);
}

LoopType LoopTypeWidget::loopType() const
{
  const auto checked_button = btn_group_->checkedButton();

  if (checked_button == sitl_btn_) {
    return SITL;
  }
  else if (checked_button == hitl_btn_) {
    return HITL;
  }
  else {
    throw std::runtime_error("The checked button does not match any of the defined buttons.");
  }
}
}  // namespace sim
}  // namespace gui
