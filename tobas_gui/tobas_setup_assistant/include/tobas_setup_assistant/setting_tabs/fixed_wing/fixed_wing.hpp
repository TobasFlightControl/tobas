// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QButtonGroup>

#include <tobas_kdl/tree.hpp>
#include <tobas_qt_tools/widgets/stacked_widget.hpp>
#include <tobas_qt_tools/widgets/tab_widget.hpp>

#include "../base_setting.hpp"
#include "./coefs/aero_coefs.hpp"
#include "./coefs/coefs.hpp"
#include "./coefs/control_surfaces.hpp"
#include "./coefs/vehicle.hpp"
#include "./helper/helper.hpp"

namespace tobas
{
namespace gui
{
namespace sa
{
namespace fw
{
class FixedWingWidget : public BaseSettingWidget
{
  Q_OBJECT

  using self = FixedWingWidget;
  using super = BaseSettingWidget;

  static constexpr char kCoefsLabel[] = "Coefficients Tab";
  static constexpr char kHelperLabel[] = "Helper Tab";
  static constexpr int kTabWidth = 120;
  static constexpr int kTabHeight = 40;

public:
  explicit FixedWingWidget(const uadf::Model& uadf, const kdl::Tree& tree);

  const char* name() const override;
  const char* title() const override;
  const char* description() const override;

  void updateInternalDataStructures() override;
  void setToDefaults() override;
  bool isValid() override;

  YAML::Node dump() const override;
  void load(const YAML::Node& node) override;

  const cf::VehicleParametersWidget* vehicle() const;
  const cf::AerodynamicsCoefficientsWidget* aeroCoefs() const;
  const cf::ControlSurfacesWidget* controlSurfaces() const;

private:
  QButtonGroup* type_btn_group_;
  qt::StackedWidget* stack_;
  qt::TabWidget* tabs_;

  cf::CoefficientsWidget* coefs_;
  hp::HelperWidget* helper_;

  int cur_idx_;

  void setCurrentButtonIndex(int index);
  void setCurrentIndex(int index);
  void onSettingTypeClicked(int new_idx);
};
}  // namespace fw
}  // namespace sa
}  // namespace gui
}  // namespace tobas
