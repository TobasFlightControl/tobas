// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/setting_tabs/propulsion_system/electric/propulsion_units/aerodynamics/aerodynamics.hpp"

#include <tobas_qt_tools/cast.hpp>
#include <tobas_yaml_tools/convert/qstring.hpp>

#include "tobas_setup_assistant/setting_tabs/propulsion_system/electric/propulsion_units/aerodynamics/blade_theory.hpp"
#include "tobas_setup_assistant/setting_tabs/propulsion_system/electric/propulsion_units/aerodynamics/preset.hpp"
#include "tobas_setup_assistant/setting_tabs/propulsion_system/electric/propulsion_units/aerodynamics/thrust_stand.hpp"
#include "tobas_setup_assistant/setting_tabs/propulsion_system/electric/propulsion_units/aerodynamics/uiuc.hpp"

namespace tobas
{
namespace gui
{
namespace sa
{
namespace propulsion
{
namespace electric
{
namespace
{
constexpr char kMethodNameKey[] = "method_name";
}  // namespace

AerodynamicsWidget::AerodynamicsWidget(const PropellerWidget* propeller) : propeller_(propeller)
{
  const auto rows = new QVBoxLayout();
  setLayout(rows);

  method_name_ = new qt::ComboBox();

  const auto preset = new AerodynamicsWidget_Preset(propeller);
  const auto thrust_stand = new AerodynamicsWidget_ThrustStand();
  const auto uiuc = new AerodynamicsWidget_UIUC(propeller);

  methods_ = new qt::StackedWidget();
  methods_->addWidget(preset);
  methods_->addWidget(thrust_stand);
  methods_->addWidget(uiuc);

  for (int i = 0; i < methods_->count(); ++i) {
    const auto method = qt::qPointerCast<AerodynamicsWidget_Base>(methods_->widget(i));
    method_name_->addItem(method->name());
  }

  // Layout
  rows->addWidget(method_name_);
  rows->addWidget(methods_);

  // Connection
  connect(
    method_name_, qOverload<int>(&qt::ComboBox::currentIndexChanged), methods_, &qt::StackedWidget::setCurrentIndex);
}

void AerodynamicsWidget::setToDefaults()
{
  for (int i = 0; i < methods_->count(); ++i) {
    const auto method = qt::qPointerCast<AerodynamicsWidget_Base>(methods_->widget(i));
    method->setToDefaults();
  }

  method_name_->setCurrentIndex(0);
}

const char* AerodynamicsWidget::name() const
{
  return "Aerodynamics";
}

bool AerodynamicsWidget::isValid()
{
  if (!selected()->isValid()) {
    return false;
  }

  return true;
}

void AerodynamicsWidget::copyFrom(const BaseSelectedLinkSettingWidget* src)
{
  const auto derived = qt::qConstPointerCast<AerodynamicsWidget>(src);

  method_name_->setCurrentIndex(derived->method_name_->currentIndex());

  for (int i = 0; i < methods_->count(); ++i) {
    const auto des_method = qt::qPointerCast<AerodynamicsWidget_Base>(methods_->widget(i));
    const auto src_method = qt::qConstPointerCast<AerodynamicsWidget_Base>(derived->methods_->widget(i));
    des_method->copyFrom(src_method);
  }
}

YAML::Node AerodynamicsWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[kMethodNameKey] = method_name_->currentText();

  for (int i = 0; i < methods_->count(); ++i) {
    const auto method = qt::qConstPointerCast<AerodynamicsWidget_Base>(methods_->widget(i));
    node[method->name()] = method->dump();
  }

  return node;
}

void AerodynamicsWidget::load(const YAML::Node& node)
{
  method_name_->setCurrentText(node[kMethodNameKey].as<QString>());

  for (int i = 0; i < methods_->count(); ++i) {
    const auto method = qt::qPointerCast<AerodynamicsWidget_Base>(methods_->widget(i));
    method->load(node[method->name()]);
  }
}

double AerodynamicsWidget::motorConst() const
{
  return selected()->motorConst();
}

double AerodynamicsWidget::momentConst() const
{
  return selected()->momentConst();
}

double AerodynamicsWidget::dragConst() const
{
  const BladeTheory blade(
    propeller_->numBlades(), propeller_->radius(), propeller_->meanChord(), propeller_->pitchAngle());
  return blade.dragConst();
}

AerodynamicsWidget_Base* AerodynamicsWidget::selected()
{
  return qt::qPointerCast<AerodynamicsWidget_Base>(methods_->currentWidget());
}

const AerodynamicsWidget_Base* AerodynamicsWidget::selected() const
{
  return qt::qConstPointerCast<AerodynamicsWidget_Base>(methods_->currentWidget());
}
}  // namespace electric
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
}  // namespace tobas
