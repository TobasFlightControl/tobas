// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "./aero_coefs.hpp"
#include "./control_surfaces.hpp"

#include <tobas_qt_tools/widgets/scroll_area.hpp>

namespace tobas
{
namespace gui
{
namespace sa
{
namespace fw
{
namespace cf
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

  AerodynamicsCoefficientsWidget* aeroCoefs() const;
  ControlSurfacesWidget* controlSurfaces() const;

private:
  AerodynamicsCoefficientsWidget* aero_coefs_;
  ControlSurfacesWidget* control_surfaces_;
};
}  // namespace cf
}  // namespace fw
}  // namespace sa
}  // namespace gui
}  // namespace tobas
