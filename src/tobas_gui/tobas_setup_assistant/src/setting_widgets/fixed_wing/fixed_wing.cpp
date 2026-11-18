// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/setting_tabs/fixed_wing/fixed_wing.hpp"

#include <QVBoxLayout>

#include <tobas_qt_tools/widgets/tab_widget.hpp>

namespace tobas::gui::sa::fw
{
namespace
{
constexpr char kVehicleLabel[] = "Vehicle Parameters";
constexpr char kAeroCoefsLabel[] = "Aerodynamic Coefficients";
constexpr char kControlSurfacesLabel[] = "Control Surfaces";
}  // namespace

FixedWingWidget::FixedWingWidget(const uadf::Model& _uadf)
{
  vehicle_ = new VehicleWidget();
  aero_coefs_ = new AerodynamicsCoefficientsWidget();
  control_surfaces_ = new ControlSurfacesWidget(_uadf);

  const auto tabs = new qt::TabWidget();
  tabs->setTabWidth(200);
  tabs->enableWheelEvent(false);
  tabs->setTabPosition(QTabWidget::North);

  tabs->addTab(vehicle_, kVehicleLabel);
  tabs->addTab(aero_coefs_, kAeroCoefsLabel);
  tabs->addTab(control_surfaces_, kControlSurfacesLabel);

  const auto rows = new QVBoxLayout(this);
  rows->addWidget(tabs);
}

const char* FixedWingWidget::name() const
{
  return "Fixed Wing";
}

const char* FixedWingWidget::title() const
{
  return "Configure Fixed-Wing Parameters";
}

const char* FixedWingWidget::description() const
{
  return "Configure the fixed-wing geometry, aerodynamic coefficients, and control-surface derivatives. "
         "Use the same reference wing area, span, and mean aerodynamic chord as the aerodynamic data. "
         "Moment coefficients are referenced to the aerodynamic center specified in Vehicle Parameters. "
         "<a href="
         "'https://vspu.larc.nasa.gov/training-content/chapter-3-model-analysis-in-openvsp/vspaero-basics'"
         ">VSPAERO</a> "
         "stability output files (.stab) can be imported in the Aerodynamic Coefficients tab.";
}

void FixedWingWidget::updateInternalDataStructures()
{
  vehicle_->updateInternalDataStructures();
  aero_coefs_->updateInternalDataStructures();
  control_surfaces_->updateInternalDataStructures();
}

void FixedWingWidget::setToDefaults()
{
  vehicle_->setToDefaults();
  aero_coefs_->setToDefaults();
  control_surfaces_->setToDefaults();
}

bool FixedWingWidget::isValid()
{
  if (!vehicle_->isValid()) {
    return false;
  }
  if (!aero_coefs_->isValid()) {
    return false;
  }
  if (!control_surfaces_->isValid()) {
    return false;
  }

  return true;
}

YAML::Node FixedWingWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[kVehicleLabel] = vehicle_->dump();
  node[kAeroCoefsLabel] = aero_coefs_->dump();
  node[kControlSurfacesLabel] = control_surfaces_->dump();

  return node;
}

void FixedWingWidget::load(const YAML::Node& _node)
{
  vehicle_->load(_node[kVehicleLabel]);
  aero_coefs_->load(_node[kAeroCoefsLabel]);
  control_surfaces_->load(_node[kControlSurfacesLabel]);
}

const VehicleWidget* FixedWingWidget::vehicle() const
{
  return vehicle_;
}

const AerodynamicsCoefficientsWidget* FixedWingWidget::aeroCoefs() const
{
  return aero_coefs_;
}

const ControlSurfacesWidget* FixedWingWidget::controlSurfaces() const
{
  return control_surfaces_;
}
}  // namespace tobas::gui::sa::fw
