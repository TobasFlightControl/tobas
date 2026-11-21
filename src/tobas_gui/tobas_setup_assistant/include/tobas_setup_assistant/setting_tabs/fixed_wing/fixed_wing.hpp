// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QButtonGroup>

#include <tobas_kdl/tree.hpp>
#include <tobas_qt_tools/widgets/stacked_widget.hpp>

#include "../base_setting.hpp"
#include "./geometry_based/geometry_based.hpp"
#include "./manual/manual.hpp"

namespace tobas::gui::sa::fw
{
class FixedWingWidget : public BaseSettingWidget
{
  Q_OBJECT

public:
  enum Type
  {
    kGEOMETRY_BASED,
    kMANUAL,
  };

  explicit FixedWingWidget(const uadf::Model& _uadf, const kdl::Tree& _tree);

  const char* name() const override;
  const char* title() const override;
  const char* description() const override;

  void updateInternalDataStructures() override;
  void setToDefaults() override;
  bool isValid() override;

  YAML::Node dump() const override;
  void load(const YAML::Node& _node) override;

  gb::Calculator* calculator() const;
  const mn::VehicleWidget* vehicle() const;
  const mn::AerodynamicsCoefficientsWidget* aeroCoefs() const;
  const mn::ControlSurfacesWidget* controlSurfaces() const;
  Type type() const;

private:
  QButtonGroup* type_btn_group_;
  qt::StackedWidget* stack_;
  qt::TabWidget* tabs_;

  mn::ManualWidget* manual_;
  gb::GeometryBasedWidget* geometry_based_;

  int cur_idx_;

  void setCurrentButtonIndex(int index);
  void setCurrentIndex(int index);
  void onSettingTypeClicked(int new_idx);
};
}  // namespace tobas::gui::sa::fw
