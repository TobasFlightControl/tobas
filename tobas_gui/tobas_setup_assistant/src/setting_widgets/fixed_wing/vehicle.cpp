// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/setting_tabs/fixed_wing/vehicle.hpp"

#include <tobas_qt_tools/layouts/form_layout.hpp>
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
namespace
{
// Keep the existing serialized keys independent of the form labels.
constexpr char kWingSurfaceKey[] = "Wing Surface";
constexpr char kWingSpanKey[] = "Wing Span";
constexpr char kMacKey[] = "Mean Aerodynamic Chord";
constexpr char kAerodynamicCenterKey[] = "Aerodynamic Center";
constexpr char kAlphaLimitKey[] = "Limitation of Angle of Attack";
}  // namespace

VehicleParametersWidget::VehicleParametersWidget()
{
  setFrameShape(QFrame::NoFrame);

  const auto form = new qt::FormLayout();
  setLayout(form);

  wing_surface_ = new qt::DoubleSpinBox();
  wing_surface_->setDecimals(3);
  wing_surface_->setMinimum(1e-3);
  wing_surface_->setValue(0.47);
  wing_surface_->setSuffix(" m²");
  form->addRow(kWingSurfaceKey, wing_surface_);
  form->labelForField(wing_surface_)->setToolTip(wing_surface_->toolTip());

  wing_span_ = new qt::DoubleSpinBox();
  wing_span_->setDecimals(3);
  wing_span_->setMinimum(1e-3);
  wing_span_->setValue(2.59);
  wing_span_->setSuffix(" m");
  form->addRow(kWingSpanKey, wing_span_);
  form->labelForField(wing_span_)->setToolTip(wing_span_->toolTip());

  mac_ = new qt::DoubleSpinBox();
  mac_->setDecimals(3);
  mac_->setMinimum(1e-3);
  mac_->setValue(0.18);
  mac_->setSuffix(" m");
  form->addRow(kMacKey, mac_);
  form->labelForField(mac_)->setToolTip(mac_->toolTip());

  aerodynamic_center_ = new qt::Vector3dEditHorizontal();
  aerodynamic_center_->setDecimals(3);
  aerodynamic_center_->setVector({ 0.1, 0.0, 0.0 });
  aerodynamic_center_->setSuffix(" m");
  form->addRow(kAerodynamicCenterKey, aerodynamic_center_);
  form->labelForField(aerodynamic_center_)->setToolTip(aerodynamic_center_->toolTip());

  alpha_limit_ = new DoubleRangeWidget();
  alpha_limit_->setDecimals(3);
  alpha_limit_->setValue({ -0.27, 0.27 });
  alpha_limit_->setSuffix(" rad");
  form->addRow(kAlphaLimitKey, alpha_limit_);
  form->labelForField(alpha_limit_)->setToolTip(alpha_limit_->toolTip());
}

void VehicleParametersWidget::updateInternalDataStructures()
{
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

  node[kWingSurfaceKey] = yaml::format(wing_surface_->value());
  node[kWingSpanKey] = yaml::format(wing_span_->value());
  node[kMacKey] = yaml::format(mac_->value());
  node[kAerodynamicCenterKey] = aerodynamic_center_->vector();
  node[kAlphaLimitKey] = alpha_limit_->value();

  return node;
}

void VehicleParametersWidget::load(const YAML::Node& node)
{
  wing_surface_->setValue(node[kWingSurfaceKey].as<double>());
  wing_span_->setValue(node[kWingSpanKey].as<double>());
  mac_->setValue(node[kMacKey].as<double>());
  aerodynamic_center_->setVector(node[kAerodynamicCenterKey].as<Eigen::Vector3d>());
  alpha_limit_->setValue(node[kAlphaLimitKey].as<st::Range<double>>());
}

double VehicleParametersWidget::wingSurface() const
{
  return wing_surface_->value();
}

double VehicleParametersWidget::wingSpan() const
{
  return wing_span_->value();
}

double VehicleParametersWidget::mac() const
{
  return mac_->value();
}

Eigen::Vector3d VehicleParametersWidget::aerodynamicCenter() const
{
  return aerodynamic_center_->vector();
}

st::Range<double> VehicleParametersWidget::alphaLimit() const
{
  return alpha_limit_->value();
}
}  // namespace fw
}  // namespace sa
}  // namespace gui
}  // namespace tobas
