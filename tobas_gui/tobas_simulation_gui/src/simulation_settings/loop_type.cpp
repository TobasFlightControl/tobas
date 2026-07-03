// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_simulation_gui/simulation_settings/loop_type.hpp"

#include <QVBoxLayout>

#include <tobas_gui_common/constants.hpp>
#include <tobas_qt_tools/widgets/label.hpp>

namespace tobas
{
namespace gui
{
namespace sim
{
LoopTypeWidget::LoopTypeWidget()
{
  btn_group_ = new QButtonGroup(this);
  btn_group_->setExclusive(true);

  // Long option text does not fit within 500 px.
  sitl_btn_ = new QRadioButton("SITL (Simulation in the Loop)");
  hitl_btn_ = new QRadioButton("HITL (Hardware in the Loop)");

  hitl_btn_->setEnabled(false);  // TODO: Enable this after HITL is supported.

  btn_group_->addButton(sitl_btn_);
  btn_group_->addButton(hitl_btn_);

  // Default
  sitl_btn_->setChecked(true);

  // Layout
  const auto rows = new QVBoxLayout();
  rows->addWidget(new qt::Label("Simulation Type", cmn::kLabelPSize, QFont::Bold));
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
}  // namespace tobas
