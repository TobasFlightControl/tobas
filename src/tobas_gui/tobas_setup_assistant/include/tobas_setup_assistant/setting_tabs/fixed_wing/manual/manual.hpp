// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "./aero_coefs.hpp"
#include "./control_surfaces.hpp"
#include "./vehicle.hpp"

namespace tobas::gui::sa::fw::mn
{
class ManualWidget : public qt::ScrollArea
{
  Q_OBJECT

public:
  explicit ManualWidget(const uadf::Model& _uadf);

  const char* name() const;
  void updateInternalDataStructures();
  void setToDefaults();
  bool isValid();

  YAML::Node dump() const;
  void load(const YAML::Node& _node);

  VehicleWidget* vehicle() const;
  AerodynamicsCoefficientsWidget* aeroCoefs() const;
  ControlSurfacesWidget* controlSurfaces() const;

private:
  VehicleWidget* vehicle_;
  AerodynamicsCoefficientsWidget* aero_coefs_;
  ControlSurfacesWidget* control_surfaces_;
};
}  // namespace tobas::gui::sa::fw::mn
