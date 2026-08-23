// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_kdl/tree.hpp>
#include <tobas_qt_tools/widgets/tab_widget.hpp>

#include "../base_setting.hpp"
#include "./coefs/aero_coefs.hpp"
#include "./coefs/coefs.hpp"
#include "./coefs/control_surfaces.hpp"
#include "./helper/helper.hpp"
#include "./vehicle.hpp"

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

  static constexpr char kVehicleLabel[] = "Vehicle Parameters";
  static constexpr char kAeroCoefsLabel[] = "Aerodynamic Coefficients";
  static constexpr char kControlSurfacesLabel[] = "Control Surfaces";
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

  const VehicleParametersWidget* vehicle() const;
  const AerodynamicsCoefficientsWidget* aeroCoefs() const;
  const ControlSurfacesWidget* controlSurfaces() const;

private:
  qt::TabWidget* tabs_;

  VehicleParametersWidget* vehicle_;
  CoefficientsWidget* coefs_;
  hp::HelperWidget* helper_;
};
}  // namespace fw
}  // namespace sa
}  // namespace gui
}  // namespace tobas
