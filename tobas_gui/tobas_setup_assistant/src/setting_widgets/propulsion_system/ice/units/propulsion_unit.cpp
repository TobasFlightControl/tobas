// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/setting_tabs/propulsion_system/ice/propulsion_units/propulsion_unit.hpp"

#include <tobas_qt_tools/cast.hpp>
#include <tobas_qt_tools/util.hpp>

namespace tobas
{
namespace gui
{
namespace sa
{
namespace propulsion
{
namespace ice
{
namespace
{
constexpr int kButtonWidth = 125;
constexpr int kButtonHeight = 50;
constexpr int kTabWidth = 120;
constexpr int kTabHeight = 40;
}  // namespace

PropulsionUnitWidget::PropulsionUnitWidget()
{
  const auto rows = new QVBoxLayout();
  setLayout(rows);

  const auto button_cols = new QHBoxLayout();
  rows->addLayout(button_cols);

  copy_to_all_btn_ = new QPushButton("Copy To All");
  copy_to_all_btn_->setFixedSize(kButtonWidth, kButtonHeight);
  qt::addWidgetCenter(copy_to_all_btn_, rows);

  button_cols->addStretch();

  tabs_ = new qt::TabWidget();
  tabs_->enableWheelEvent(false);
  tabs_->setTabSize(kTabWidth, kTabHeight);
  rows->addWidget(tabs_);

  transmission_ = new TransmissionWidget();
  propeller_ = new PropellerWidget();
  aerodynamics_ = new AerodynamicsWidget(propeller_);

  tabs_->addTab(transmission_, transmission_->name());
  tabs_->addTab(propeller_, propeller_->name());
  tabs_->addTab(aerodynamics_, aerodynamics_->name());

  // Connection
  connect(copy_to_all_btn_, &QPushButton::clicked, [this]() { Q_EMIT copyToAllButtonClicked(); });
}

void PropulsionUnitWidget::setToDefaults()
{
  for (int i = 0; i < tabs_->count(); ++i) {
    const auto widget = qt::qPointerCast<BaseSelectedLinkSettingWidget>(tabs_->widget(i));
    widget->setToDefaults();
  }
}

bool PropulsionUnitWidget::isValid()
{
  for (int i = 0; i < tabs_->count(); ++i) {
    const auto widget = qt::qPointerCast<BaseSelectedLinkSettingWidget>(tabs_->widget(i));
    if (!widget->isValid()) {
      return false;
    }
  }

  return true;
}

void PropulsionUnitWidget::copyFrom(const PropulsionUnitWidget* src)
{
  for (int i = 0; i < tabs_->count(); ++i) {
    const auto des_widget = qt::qPointerCast<BaseSelectedLinkSettingWidget>(tabs_->widget(i));
    const auto src_widget = qt::qConstPointerCast<const BaseSelectedLinkSettingWidget>(src->tabs_->widget(i));
    des_widget->copyFrom(src_widget);
  }
}

YAML::Node PropulsionUnitWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  for (int i = 0; i < tabs_->count(); ++i) {
    const auto widget = qt::qConstPointerCast<const BaseSelectedLinkSettingWidget>(tabs_->widget(i));
    node[widget->name()] = widget->dump();
  }

  return node;
}

void PropulsionUnitWidget::load(const YAML::Node& node)
{
  for (int i = 0; i < tabs_->count(); ++i) {
    const auto widget = qt::qPointerCast<BaseSelectedLinkSettingWidget>(tabs_->widget(i));
    widget->load(node[widget->name()]);
  }
}

const TransmissionWidget* PropulsionUnitWidget::transmission() const
{
  return transmission_;
}

const PropellerWidget* PropulsionUnitWidget::propeller() const
{
  return propeller_;
}

const AerodynamicsWidget* PropulsionUnitWidget::aerodynamics() const
{
  return aerodynamics_;
}
}  // namespace ice
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
}  // namespace tobas
