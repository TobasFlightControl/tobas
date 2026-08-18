// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "../../base_setting.hpp"
#include "./aero_coefs.hpp"
#include "./control_surfaces.hpp"

namespace tobas
{
namespace gui
{
namespace sa
{
namespace fw
{
class CoefficientsWidget : public qt::ScrollArea
{
  Q_OBJECT

  using self = CoefficientsWidget;
  using super = qt::ScrollArea;

  static constexpr char kAeroCoefsLabel[] = "Aerodynamic Coefficients";
  static constexpr char kControlSurfacesLabel[] = "Control Surfaces";

public:
  explicit CoefficientsWidget(const uadf::Model& uadf);

  const char* name() const;
  void updateInternalDataStructures();
  void setToDefaults();
  bool isValid();

  YAML::Node dump() const;
  void load(const YAML::Node& node);

  const AerodynamicsCoefficientsWidget* aeroCoefs() const;
  const ControlSurfacesWidget* controlSurfaces() const;

private:
  QSettings settings_store_;

  qt::FormLayout* form_;

  AerodynamicsCoefficientsWidget* aero_coefs_;
  ControlSurfacesWidget* control_surfaces_;
};
}  // namespace fw
}  // namespace sa
}  // namespace gui
}  // namespace tobas
