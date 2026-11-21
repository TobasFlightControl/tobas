// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <yaml-cpp/yaml.h>

#include <tobas_qt_tools/layouts/form_layout.hpp>
#include <tobas_qt_tools/widgets/double_spin_box.hpp>
#include <tobas_qt_tools/widgets/scroll_area.hpp>
#include <tobas_qt_tools/widgets/vector3d_edit_horizontal.hpp>

#include "tobas_setup_assistant/utils/double_range.hpp"

namespace tobas
{
namespace gui
{
namespace sa
{
namespace fw
{
class VehicleParametersWidget : public qt::ScrollArea
{
  Q_OBJECT

  using self = VehicleParametersWidget;
  using super = qt::ScrollArea;

public:
  explicit VehicleParametersWidget();

  void updateInternalDataStructures();
  bool isValid();

  YAML::Node dump() const;
  void load(const YAML::Node& node);

  double wingSurface() const;
  double wingSpan() const;
  double mac() const;
  Eigen::Vector3d aerodynamicCenter() const;
  st::Range<double> alphaLimit() const;

private:
  qt::DoubleSpinBox* wing_surface_;
  qt::DoubleSpinBox* wing_span_;
  qt::DoubleSpinBox* mac_;
  qt::Vector3dEditHorizontal* aerodynamic_center_;
  DoubleRangeWidget* alpha_limit_;
};
}  // namespace fw
}  // namespace sa
}  // namespace gui
}  // namespace tobas
