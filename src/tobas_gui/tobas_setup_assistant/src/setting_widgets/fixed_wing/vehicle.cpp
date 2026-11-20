// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/setting_tabs/fixed_wing/vehicle.hpp"

#include <QFrame>

#include <tobas_qt_tools/layouts/form_layout.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_yaml_tools/convert/eigen.hpp>
#include <tobas_yaml_tools/convert/range.hpp>
#include <tobas_yaml_tools/format.hpp>

namespace tobas::gui::sa::fw
{
namespace
{
constexpr char kWingSurfaceLabel[] = "Wing Area";
constexpr char kWingSpanLabel[] = "Wing Span";
constexpr char kMacLabel[] = "Mean Aerodynamic Chord";
constexpr char kMomentReferencePointLabel[] = "Moment Reference Point";
constexpr char kAlphaLimitLabel[] = "Angle-of-Attack Limits";
}  // namespace

VehicleWidget::VehicleWidget()
{
  setFrameShape(QFrame::NoFrame);

  wing_surface_ = new qt::DoubleSpinBox();
  wing_surface_->setToolTip("Reference wing area in square meters, used to scale aerodynamic forces and moments.");
  wing_surface_->setDecimals(3);
  wing_surface_->setMinimum(1e-3);
  wing_surface_->setSuffix(" m²");

  wing_span_ = new qt::DoubleSpinBox();
  wing_span_->setToolTip("Reference wing span in meters, used to scale rolling and yawing moments.");
  wing_span_->setDecimals(3);
  wing_span_->setMinimum(1e-3);
  wing_span_->setSuffix(" m");

  mac_ = new qt::DoubleSpinBox();
  mac_->setToolTip("Mean aerodynamic chord in meters, used as the reference length for pitching moments.");
  mac_->setDecimals(3);
  mac_->setMinimum(1e-3);
  mac_->setSuffix(" m");

  moment_reference_point_ = new qt::Vector3dEditHorizontal();
  moment_reference_point_->setToolTip(
    "Moment reference point position in meters relative to the airframe origin. "
    "Coordinates use the forward-left-up (FLU) frame.");
  moment_reference_point_->setDecimals(3);
  moment_reference_point_->setSuffix(" m");

  alpha_limit_ = new DoubleRangeWidget();
  alpha_limit_->setToolTip(
    "Lower and upper limits of the modeled angle of attack in radians, "
    "bounded by the negative and positive stall angles. "
    "The lower limit must not exceed the upper limit.");
  alpha_limit_->setDecimals(3);
  alpha_limit_->setSuffix(" rad");

  const auto form = new qt::FormLayout();
  setLayout(form);  // Do not pass `this` to the layout constructor.
  form->addRow(kWingSurfaceLabel, wing_surface_);
  form->labelForField(wing_surface_)->setToolTip(wing_surface_->toolTip());
  form->addRow(kWingSpanLabel, wing_span_);
  form->labelForField(wing_span_)->setToolTip(wing_span_->toolTip());
  form->addRow(kMacLabel, mac_);
  form->labelForField(mac_)->setToolTip(mac_->toolTip());
  form->addRow(kMomentReferencePointLabel, moment_reference_point_);
  form->labelForField(moment_reference_point_)->setToolTip(moment_reference_point_->toolTip());
  form->addRow(kAlphaLimitLabel, alpha_limit_);
  form->labelForField(alpha_limit_)->setToolTip(alpha_limit_->toolTip());
}

void VehicleWidget::updateInternalDataStructures()
{
}

void VehicleWidget::setToDefaults()
{
  wing_surface_->setValue(0.47);
  wing_span_->setValue(2.59);
  mac_->setValue(0.18);
  moment_reference_point_->setVector({ 0.1, 0.0, 0.0 });
  alpha_limit_->setValue({ -0.27, 0.27 });
}

bool VehicleWidget::isValid()
{
  if (!alpha_limit_->isValid()) {
    qt::qWarnBox(this, "Invalid angle-of-attack limits.");
    return false;
  }

  return true;
}

YAML::Node VehicleWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[kWingSurfaceLabel] = yaml::format(wing_surface_->value());
  node[kWingSpanLabel] = yaml::format(wing_span_->value());
  node[kMacLabel] = yaml::format(mac_->value());
  node[kMomentReferencePointLabel] = moment_reference_point_->vector();
  node[kAlphaLimitLabel] = alpha_limit_->value();

  return node;
}

void VehicleWidget::load(const YAML::Node& _node)
{
  wing_surface_->setValue(_node[kWingSurfaceLabel].as<double>());
  wing_span_->setValue(_node[kWingSpanLabel].as<double>());
  mac_->setValue(_node[kMacLabel].as<double>());
  moment_reference_point_->setVector(_node[kMomentReferencePointLabel].as<Eigen::Vector3d>());
  alpha_limit_->setValue(_node[kAlphaLimitLabel].as<st::Range<double>>());
}

double VehicleWidget::wingSurface() const
{
  return wing_surface_->value();
}

double VehicleWidget::wingSpan() const
{
  return wing_span_->value();
}

double VehicleWidget::mac() const
{
  return mac_->value();
}

Eigen::Vector3d VehicleWidget::momentReferencePoint() const
{
  return moment_reference_point_->vector();
}

st::Range<double> VehicleWidget::alphaLimit() const
{
  return alpha_limit_->value();
}
}  // namespace tobas::gui::sa::fw
