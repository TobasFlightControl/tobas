// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <yaml-cpp/yaml.h>
#include <eigen3/Eigen/Core>

#include <tobas_qt_tools/widgets/double_spin_box.hpp>
#include <tobas_qt_tools/widgets/scroll_area.hpp>
#include <tobas_qt_tools/widgets/vector3d_edit_horizontal.hpp>
#include <tobas_std_tools/range.hpp>

#include "tobas_setup_assistant/utils/double_range.hpp"

namespace tobas::gui::sa::fw::mn
{
class VehicleWidget : public qt::ScrollArea
{
  Q_OBJECT

public:
  explicit VehicleWidget();

  void updateInternalDataStructures();
  void setToDefaults();
  bool isValid();

  YAML::Node dump() const;
  void load(const YAML::Node& _node);

  // getters
  double wingSurface() const;
  double wingSpan() const;
  double mac() const;
  Eigen::Vector3d momentReferencePoint() const;
  st::Range<double> alphaLimit() const;
  // setters
  void wingSurface(const double& value);
  void wingSpan(const double& value);
  void mac(const double& value);
  void momentReferencePoint(const Eigen::Vector3d& value);
  void alphaLimit(const st::Range<double>& value);

private:
  qt::DoubleSpinBox* wing_surface_;
  qt::DoubleSpinBox* wing_span_;
  qt::DoubleSpinBox* mac_;
  qt::Vector3dEditHorizontal* moment_reference_point_;
  DoubleRangeWidget* alpha_limit_;
};
}  // namespace tobas::gui::sa::fw::mn
