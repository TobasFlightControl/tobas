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
constexpr char kWingSurfaceLabel[] = "Wing Surface";
constexpr char kWingSpanLabel[] = "Wing Span";
constexpr char kMacLabel[] = "Mean Aerodynamic Chord";
constexpr char kAerodynamicCenterLabel[] = "Aerodynamic Center";
constexpr char kAlphaLimitLabel[] = "Limitation of Angle of Attack";
}  // namespace

VehicleWidget::VehicleWidget()
{
  setFrameShape(QFrame::NoFrame);

  wing_surface_ = new qt::DoubleSpinBox();
  wing_surface_->setDecimals(3);
  wing_surface_->setMinimum(1e-3);
  wing_surface_->setSuffix(" m²");

  wing_span_ = new qt::DoubleSpinBox();
  wing_span_->setDecimals(3);
  wing_span_->setMinimum(1e-3);
  wing_span_->setSuffix(" m");

  mac_ = new qt::DoubleSpinBox();
  mac_->setDecimals(3);
  mac_->setMinimum(1e-3);
  mac_->setSuffix(" m");

  aerodynamic_center_ = new qt::Vector3dEditHorizontal();
  aerodynamic_center_->setDecimals(3);
  aerodynamic_center_->setSuffix(" m");

  alpha_limit_ = new DoubleRangeWidget();
  alpha_limit_->setDecimals(3);
  alpha_limit_->setSuffix(" rad");

  const auto form = new qt::FormLayout();
  setLayout(form);  // Do not pass `this` to the layout constructor.
  form->addRow(kWingSurfaceLabel, wing_surface_);
  form->addRow(kWingSpanLabel, wing_span_);
  form->addRow(kMacLabel, mac_);
  form->addRow(kAerodynamicCenterLabel, aerodynamic_center_);
  form->addRow(kAlphaLimitLabel, alpha_limit_);
}

void VehicleWidget::updateInternalDataStructures()
{
}

void VehicleWidget::setToDefaults()
{
  wing_surface_->setValue(0.47);
  wing_span_->setValue(2.59);
  mac_->setValue(0.18);
  aerodynamic_center_->setVector({ 0.1, 0.0, 0.0 });
  alpha_limit_->setValue({ -0.27, 0.27 });
}

bool VehicleWidget::isValid()
{
  if (!alpha_limit_->isValid()) {
    qt::qWarnBox(this, "Invalid limitation of angle of attack.");
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
  node[kAerodynamicCenterLabel] = aerodynamic_center_->vector();
  node[kAlphaLimitLabel] = alpha_limit_->value();

  return node;
}

void VehicleWidget::load(const YAML::Node& _node)
{
  wing_surface_->setValue(_node[kWingSurfaceLabel].as<double>());
  wing_span_->setValue(_node[kWingSpanLabel].as<double>());
  mac_->setValue(_node[kMacLabel].as<double>());
  aerodynamic_center_->setVector(_node[kAerodynamicCenterLabel].as<Eigen::Vector3d>());
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

Eigen::Vector3d VehicleWidget::aerodynamicCenter() const
{
  return aerodynamic_center_->vector();
}

st::Range<double> VehicleWidget::alphaLimit() const
{
  return alpha_limit_->value();
}
}  // namespace tobas::gui::sa::fw
