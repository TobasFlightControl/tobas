// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_qt_tools/widgets/scroll_area.hpp>

#include <tobas_setup_assistant/setting_tabs/fixed_wing/manual/aero_coefs.hpp>
#include <tobas_setup_assistant/setting_tabs/fixed_wing/manual/control_surfaces.hpp>
#include <tobas_setup_assistant/setting_tabs/fixed_wing/manual/vehicle.hpp>

namespace tobas
{
namespace gui
{
namespace sa
{
namespace fw
{
namespace mn
{
class ManualWidget : public qt::ScrollArea
{
  Q_OBJECT

  using self = ManualWidget;
  using super = qt::ScrollArea;

  static constexpr char kVehicleLabel[] = "Vehicle Parameters";
  static constexpr char kAeroCoefsLabel[] = "Aerodynamic Coefficients";
  static constexpr char kControlSurfacesLabel[] = "Control Surfaces";

public:
  explicit ManualWidget(const uadf::Model& uadf);

  const char* name() const;
  void updateInternalDataStructures();
  void setToDefaults();
  bool isValid();

  YAML::Node dump() const;
  void load(const YAML::Node& node);

  VehicleParametersWidget* vehicle() const;
  AerodynamicsCoefficientsWidget* aeroCoefs() const;
  ControlSurfacesWidget* controlSurfaces() const;

private:
  VehicleParametersWidget* vehicle_;
  AerodynamicsCoefficientsWidget* aero_coefs_;
  ControlSurfacesWidget* control_surfaces_;
};
}  // namespace mn
}  // namespace fw
}  // namespace sa
}  // namespace gui
}  // namespace tobas
