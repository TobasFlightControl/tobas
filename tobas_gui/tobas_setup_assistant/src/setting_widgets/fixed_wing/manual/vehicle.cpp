// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/setting_tabs/fixed_wing/manual/vehicle.hpp"

#include <tobas_qt_tools/message.hpp>
#include <tobas_yaml_tools/convert/eigen.hpp>
#include <tobas_yaml_tools/convert/range.hpp>
#include <tobas_yaml_tools/format.hpp>

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
VehicleParametersWidget::VehicleParametersWidget()
{
  const auto rows = new QVBoxLayout();
  setLayout(rows);

  wing_surface_ = new ParamGetterWidget_DoubleSpinBox("Wing Surface", "");
  wing_surface_->setDecimals(3);
  wing_surface_->setMinimum(1e-3);
  wing_surface_->setSuffix(" m^2");
  rows->addWidget(wing_surface_);

  wing_span_ = new ParamGetterWidget_DoubleSpinBox("Wing Span", "");
  wing_span_->setDecimals(3);
  wing_span_->setMinimum(1e-3);
  wing_span_->setSuffix(" m");
  rows->addWidget(wing_span_);

  mac_ = new ParamGetterWidget_DoubleSpinBox("Mean Aerodynamic Chord", "");
  mac_->setDecimals(3);
  mac_->setMinimum(1e-3);
  mac_->setSuffix(" m");
  rows->addWidget(mac_);

  moment_reference_point_ = new ParamGetterWidget_Vector3d("Moment Reference Point", "The coordinates viewed in the origin-centered FLU coordinate system. We calculate the stability derivatives about this point.");
  moment_reference_point_->setDecimals(3);
  moment_reference_point_->setSuffix(" m");
  rows->addWidget(moment_reference_point_);

  alpha_limit_ = new ParamGetterWidget_DoubleRange("Limitation of Angle of Attack", "");
  alpha_limit_->setDecimals(3);
  alpha_limit_->setSuffix(" rad");
  rows->addWidget(alpha_limit_);
}

void VehicleParametersWidget::updateInternalDataStructures()
{
}

void VehicleParametersWidget::setToDefaults()
{
  wing_surface_->setValue(0.47);
  wing_span_->setValue(2.59);
  mac_->setValue(0.18);
  moment_reference_point_->setValue({ 0.1, 0.0, 0.0 });
  alpha_limit_->setValue({ -0.27, 0.27 });
}

bool VehicleParametersWidget::isValid()
{
  if (!alpha_limit_->isValid()) {
    qt::qWarnBox(this, "Invalid limitation of angle of attack.");
    return false;
  }

  return true;
}

YAML::Node VehicleParametersWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[wing_surface_->name()] = yaml::format(wing_surface_->getValue());
  node[wing_span_->name()] = yaml::format(wing_span_->getValue());
  node[mac_->name()] = yaml::format(mac_->getValue());
  node[moment_reference_point_->name()] = moment_reference_point_->getValue();
  node[alpha_limit_->name()] = alpha_limit_->getValue();

  return node;
}

void VehicleParametersWidget::load(const YAML::Node& node)
{
  wing_surface_->setValue(node[wing_surface_->name()].as<double>());
  wing_span_->setValue(node[wing_span_->name()].as<double>());
  mac_->setValue(node[mac_->name()].as<double>());
  moment_reference_point_->setValue(node[moment_reference_point_->name()].as<Eigen::Vector3d>());
  alpha_limit_->setValue(node[alpha_limit_->name()].as<st::Range<double>>());
}

double VehicleParametersWidget::wingSurface() const
{
  return wing_surface_->getValue();
}

double VehicleParametersWidget::wingSpan() const
{
  return wing_span_->getValue();
}

double VehicleParametersWidget::mac() const
{
  return mac_->getValue();
}

Eigen::Vector3d VehicleParametersWidget::momentReferencePoint() const
{
  return moment_reference_point_->getValue();
}

st::Range<double> VehicleParametersWidget::alphaLimit() const
{
  return alpha_limit_->getValue();
}

void VehicleParametersWidget::wingSurface(const double& value)
{
  wing_surface_->setValue(value);
}

void VehicleParametersWidget::wingSpan(const double& value)
{
  wing_span_->setValue(value);
}

void VehicleParametersWidget::mac(const double& value)
{
  mac_->setValue(value);
}

void VehicleParametersWidget::momentReferencePoint(const Eigen::Vector3d& value)
{
  moment_reference_point_->setValue(value);
}

void VehicleParametersWidget::alphaLimit(const st::Range<double>& value)
{
  alpha_limit_->setValue(value);
}

}  // namespace mn
}  // namespace fw
}  // namespace sa
}  // namespace gui
}  // namespace tobas
