// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/setting_tabs/fixed_wing/fixed_wing.hpp"

#include <QRadioButton>

#include <tobas_gui_common/constants.hpp>
#include <tobas_qt_tools/cast.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_qt_tools/widgets/label.hpp>

namespace tobas
{
namespace gui
{
namespace sa
{
namespace fw
{
FixedWingWidget::FixedWingWidget(const uadf::Model& uadf, const kdl::Tree& tree)
{
  type_btn_group_ = new QButtonGroup(this);
  type_btn_group_->setExclusive(true);

  stack_ = new qt::StackedWidget();
  int id = 0;

  // widgets in stack
  coefs_ = new cf::CoefficientsWidget(uadf);
  helper_ = new hp::HelperWidget(uadf, tree, coefs_);

  // helper button
  const auto helper_btn = new QRadioButton(helper_->name());
  type_btn_group_->addButton(helper_btn, id++);
  addWidget(helper_btn);
  // coefs button
  const auto coefs_btn = new QRadioButton(coefs_->name());
  type_btn_group_->addButton(coefs_btn, id++);
  addWidget(coefs_btn);

  // stack
  stack_->addWidget(helper_); // helperが先
  stack_->addWidget(coefs_);
  addWidget(stack_);

  connect(type_btn_group_, &QButtonGroup::idClicked, this, &self::onSettingTypeClicked);
}

const char* FixedWingWidget::name() const
{
  return "Fixed Wing";
}

const char* FixedWingWidget::title() const
{
  return "Define Fixed Wing";
}

const char* FixedWingWidget::description() const
{
  return "Build the mathematical model for the fixed wing and its control surfaces. "
         "In addition to the general airframe specifications, "
         "supply the stability derivatives for the main wing and each control surface. "
         "<a href="
         "'https://vspu.larc.nasa.gov/training-content/chapter-3-model-analysis-in-openvsp/vspaero-basics'"
         ">VSPAERO</a> "
         "analysis results can be imported for the main wing if available.";
}

void FixedWingWidget::updateInternalDataStructures()
{
  coefs_->updateInternalDataStructures();
  helper_->updateInternalDataStructures();
}

void FixedWingWidget::setToDefaults()
{
  coefs_->setToDefaults();
  helper_->setToDefaults();

  static constexpr int kDefaultIndex = 0;
  setCurrentIndex(kDefaultIndex);
}

bool FixedWingWidget::isValid()
{
  if (!coefs_->isValid()) {
    return false;
  }
  if (helper_->isValid()) {
    return false;
  }

  return true;
}

YAML::Node FixedWingWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[kCoefsLabel] = coefs_->dump();
  node[kHelperLabel] = helper_->dump();

  return node;
}

void FixedWingWidget::load(const YAML::Node& node)
{
  coefs_->load(node[kCoefsLabel]);
  helper_->load(node[kHelperLabel]);
}

const cf::VehicleParametersWidget* FixedWingWidget::vehicle() const
{
  return coefs_->vehicle();
}

const cf::AerodynamicsCoefficientsWidget* FixedWingWidget::aeroCoefs() const
{
  return coefs_->aeroCoefs();
}

const cf::ControlSurfacesWidget* FixedWingWidget::controlSurfaces() const
{
  return coefs_->controlSurfaces();
}

void FixedWingWidget::setCurrentButtonIndex(int index)
{
  // Do nothing if the checked button does not change.
  if (type_btn_group_->checkedId() == index) {
    return;
  }

  // Get the buttons before and after switching.
  const auto old_btn = type_btn_group_->checkedButton();
  const auto new_btn = type_btn_group_->button(index);

  // Block all signals; passing nullptr is okay.
  const QSignalBlocker block_group(type_btn_group_);
  const QSignalBlocker block_old_btn(old_btn);
  const QSignalBlocker block_new_btn(new_btn);

  // Check the new button; `old_btn` is unchecked automatically because the group is exclusive.
  new_btn->setChecked(true);
}

void FixedWingWidget::setCurrentIndex(int index)
{
  setCurrentButtonIndex(index);
  stack_->setCurrentIndex(index);
  cur_idx_ = index;
}

void FixedWingWidget::onSettingTypeClicked(int new_idx)
{
  qDebug().nospace() << "FixedWingWidget::onSettingTypeChanged(" << new_idx << ")";

  if (new_idx == cur_idx_) {
    return;
  }

  // Switch propulsion-system widgets.
  stack_->setCurrentIndex(new_idx);
  cur_idx_ = new_idx;
}
}  // namespace fw
}  // namespace sa
}  // namespace gui
}  // namespace tobas
