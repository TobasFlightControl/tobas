// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <yaml-cpp/yaml.h>

#include <tobas_qt_tools/layouts/form_layout.hpp>

#include <tobas_setup_assistant/param_getters/double_range.hpp>
#include <tobas_setup_assistant/param_getters/double_spin_box.hpp>
#include <tobas_setup_assistant/param_getters/vector3d.hpp>

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
class VehicleParametersWidget : public QWidget
{
  Q_OBJECT

  using self = VehicleParametersWidget;
  using super = QWidget;

public:
  explicit VehicleParametersWidget();

  void updateInternalDataStructures();
  void setToDefaults();
  bool isValid();

  YAML::Node dump() const;
  void load(const YAML::Node& node);

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
  ParamGetterWidget_DoubleSpinBox* wing_surface_;
  ParamGetterWidget_DoubleSpinBox* wing_span_;
  ParamGetterWidget_DoubleSpinBox* mac_;
  ParamGetterWidget_Vector3d* moment_reference_point_;
  ParamGetterWidget_DoubleRange* alpha_limit_;
};
}  // namespace mn
}  // namespace fw
}  // namespace sa
}  // namespace gui
}  // namespace tobas
