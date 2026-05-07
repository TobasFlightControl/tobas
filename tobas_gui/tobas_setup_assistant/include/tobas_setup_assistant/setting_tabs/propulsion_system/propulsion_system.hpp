// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QButtonGroup>

#include <tobas_qt_tools/widgets/stacked_widget.hpp>

#include "../base_setting.hpp"
#include "./base.hpp"
#include "./electric/propulsion_system.hpp"
#include "./ice/propulsion_system.hpp"
#include "tobas_setup_assistant/signals.hpp"

namespace tobas
{
namespace gui
{
namespace sa
{
namespace propulsion
{
class PropulsionSystemWidget : public BaseSettingWidget
{
  Q_OBJECT

  using self = PropulsionSystemWidget;
  using super = BaseSettingWidget;

  static constexpr char kTypeKey[] = "propulsion_system_type";

public:
  explicit PropulsionSystemWidget(rclcpp::Node::SharedPtr node, const uadf::Model& uadf, Signals& sig);

  const char* name() const override;
  const char* title() const override;
  const char* description() const override;

  void updateInternalDataStructures() override;
  bool isValid() override;

  YAML::Node dump() const override;
  void load(const YAML::Node& node) override;

  PropulsionSystem type() const;
  int numUnits() const;

  QString linkName(int index) const;

  BasePropulsionSystemWidget* widget(int index);
  const BasePropulsionSystemWidget* widget(int index) const;

  BasePropulsionSystemWidget* selected();
  const BasePropulsionSystemWidget* selected() const;

private:
  Signals& sig_;

  QButtonGroup* type_btn_group_;
  qt::StackedWidget* propulsion_stack_;

  int cur_idx_;

  void addPropulsionSystemWidget(BasePropulsionSystemWidget* widget, int id);

  void setCurrentButtonIndex(int index);
  void setCurrentIndex(int index);

private Q_SLOTS:
  void onPropulsionTypeClicked(int new_idx);
};
};  // namespace propulsion
}  // namespace sa
}  // namespace gui
}  // namespace tobas
