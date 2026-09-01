// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/setting_tabs/propulsion_system/propulsion_system.hpp"

#include <ranges>

#include <QRadioButton>

#include <tobas_qt_tools/cast.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_yaml_tools/convert/qstring.hpp>

namespace tobas
{
namespace gui
{
namespace sa
{
namespace propulsion
{
namespace
{
constexpr int kTabWidth = 150;
constexpr int kTabHeight = 50;

constexpr char kTypeKey[] = "propulsion_system_type";
}  // namespace

PropulsionSystemWidget::PropulsionSystemWidget(const uadf::Model& uadf, Signals& sig) : sig_(sig)
{
  type_btn_group_ = new QButtonGroup(this);
  type_btn_group_->setExclusive(true);

  propulsion_stack_ = new qt::StackedWidget();

  int id = 0;
  addPropulsionSystemWidget(new electric::PropulsionSystemWidget(uadf), id++);
  addPropulsionSystemWidget(new ice::PropulsionSystemWidget(uadf), id++);

  addSpacing(50);
  addWidget(propulsion_stack_);

  connect(type_btn_group_, &QButtonGroup::idClicked, this, &self::onPropulsionTypeClicked);
}

const char* PropulsionSystemWidget::name() const
{
  return "Propulsion System";
}

const char* PropulsionSystemWidget::title() const
{
  return "Define Propulsion System";
}

const char* PropulsionSystemWidget::description() const
{
  return "Build the mathematical model for your propulsion system. "
         "Tobas supports two configurations:\n"
         "  1. Electric – battery‑powered with fixed‑pitch propellers\n"
         "  2. ICE – an internal‑combustion engine driving variable‑pitch propellers through gearboxes\n"
         "An accurate propulsion model is critical to maximizing aircraft performance. "
         "Select the appropriate architecture and enter the required parameters for each field.";
}

void PropulsionSystemWidget::updateInternalDataStructures()
{
  for (int i = 0; i < propulsion_stack_->count(); ++i) {
    const auto propulsion = widget(i);
    propulsion->updateInternalDataStructures();
  }
}

void PropulsionSystemWidget::setToDefaults()
{
  for (int i = 0; i < propulsion_stack_->count(); ++i) {
    widget(i)->setToDefaults();
  }

  constexpr int kDefaultIndex = 0;
  setCurrentIndex(kDefaultIndex);
  Q_EMIT sig_.propulsionTypeChanged(widget(kDefaultIndex)->type());
}

bool PropulsionSystemWidget::isValid()
{
  if (!selected()->isValid()) {
    return false;
  }

  return true;
}

YAML::Node PropulsionSystemWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  const auto type_btn = type_btn_group_->checkedButton();
  node[kTypeKey] = type_btn->text();

  for (int i = 0; i < propulsion_stack_->count(); ++i) {
    const auto propulsion = widget(i);
    node[propulsion->name()] = propulsion->dump();
  }

  return node;
}

void PropulsionSystemWidget::load(const YAML::Node& node)
{
  // Select and notify the propulsion-system type the same way as when the user operates it.
  const auto type_text = node[kTypeKey].as<QString>();
  for (const auto& [idx, button] : std::views::enumerate(type_btn_group_->buttons())) {
    if (button->text() == type_text) {
      setCurrentIndex(idx);
      Q_EMIT sig_.propulsionTypeChanged(widget(idx)->type());
      break;
    }
  }

  for (int i = 0; i < propulsion_stack_->count(); ++i) {
    const auto propulsion = widget(i);
    propulsion->load(node[propulsion->name()]);
  }
}

PropulsionSystem PropulsionSystemWidget::type() const
{
  return selected()->type();
}

int PropulsionSystemWidget::numUnits() const
{
  return selected()->numUnits();
}

QString PropulsionSystemWidget::linkName(int index) const
{
  return selected()->linkName(index);
}

BasePropulsionSystemWidget* PropulsionSystemWidget::widget(int index)
{
  return qt::qPointerCast<BasePropulsionSystemWidget>(propulsion_stack_->widget(index));
}

const BasePropulsionSystemWidget* PropulsionSystemWidget::widget(int index) const
{
  return qt::qConstPointerCast<BasePropulsionSystemWidget>(propulsion_stack_->widget(index));
}

BasePropulsionSystemWidget* PropulsionSystemWidget::selected()
{
  return qt::qPointerCast<BasePropulsionSystemWidget>(propulsion_stack_->currentWidget());
}

const BasePropulsionSystemWidget* PropulsionSystemWidget::selected() const
{
  return qt::qConstPointerCast<BasePropulsionSystemWidget>(propulsion_stack_->currentWidget());
}

void PropulsionSystemWidget::addPropulsionSystemWidget(BasePropulsionSystemWidget* widget, int id)
{
  const auto btn = new QRadioButton(widget->name());
  type_btn_group_->addButton(btn, id);
  addWidget(btn);
  propulsion_stack_->addWidget(widget);
}

void PropulsionSystemWidget::setCurrentButtonIndex(int index)
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

void PropulsionSystemWidget::setCurrentIndex(int index)
{
  setCurrentButtonIndex(index);
  propulsion_stack_->setCurrentIndex(index);
  cur_idx_ = index;
}

void PropulsionSystemWidget::onPropulsionTypeClicked(int new_idx)
{
  qDebug().nospace() << "PropulsionSystemWidget::onPropulsionTypeChanged(" << new_idx << ")";

  if (new_idx == cur_idx_) {
    return;
  }

  if (!qt::yesOrNo(
        this, "Changing the propulsion type will reset the wiring settings. Do you want to continue?", qt::WARN)) {
    setCurrentButtonIndex(cur_idx_);
    return;
  }

  // Switch propulsion-system widgets.
  propulsion_stack_->setCurrentIndex(new_idx);
  cur_idx_ = new_idx;

  // Notify other widgets that the propulsion-system type changed.
  Q_EMIT sig_.propulsionTypeChanged(widget(new_idx)->type());
}
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
}  // namespace tobas
