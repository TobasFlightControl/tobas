// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QButtonGroup>

#include <tobas_qt_tools/widgets/stacked_widget.hpp>
#include <tobas_uadf/model.hpp>

#include "../base_setting.hpp"
#include "./base.hpp"
#include "tobas_setup_assistant/signals.hpp"

namespace tobas::gui::sa::propulsion
{
class PropulsionSystemWidget : public BaseSettingWidget
{
  Q_OBJECT

  using self = PropulsionSystemWidget;

public:
  explicit PropulsionSystemWidget(const uadf::Model& _uadf, Signals& _sig);

  const char* name() const override;
  const char* title() const override;
  const char* description() const override;

  void updateInternalDataStructures() override;
  void setToDefaults() override;
  bool isValid() override;

  YAML::Node dump() const override;
  void load(const YAML::Node& _node) override;

  PropulsionSystem type() const;
  int numUnits() const;

  QString linkName(int _index) const;

  BasePropulsionSystemWidget* widget(int _index);
  const BasePropulsionSystemWidget* widget(int _index) const;

  BasePropulsionSystemWidget* selected();
  const BasePropulsionSystemWidget* selected() const;

private:
  Signals& sig_;

  QButtonGroup* type_btn_group_;
  qt::StackedWidget* propulsion_stack_;

  int cur_idx_;

  void addPropulsionSystemWidget(BasePropulsionSystemWidget* _widget, int _id);

  void setCurrentButtonIndex(int _index);
  void setCurrentIndex(int _index);

private Q_SLOTS:
  void onPropulsionTypeClicked(int _new_idx);
};
}  // namespace tobas::gui::sa::propulsion
