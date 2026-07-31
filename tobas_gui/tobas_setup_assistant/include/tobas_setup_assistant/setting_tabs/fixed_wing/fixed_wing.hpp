// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "../base_setting.hpp"
#include "./aero_coefs.hpp"
#include "./control_surfaces.hpp"
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

public:
  explicit FixedWingWidget(rclcpp::Node::SharedPtr node, const uadf::Model& uadf);

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
  const rclcpp::Node::SharedPtr node_;

  VehicleParametersWidget* vehicle_;
  AerodynamicsCoefficientsWidget* aero_coefs_;
  ControlSurfacesWidget* control_surfaces_;
};
}  // namespace fw
}  // namespace sa
}  // namespace gui
}  // namespace tobas
