// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/setting_tabs/fixed_wing/manual/manual.hpp"

#include <QVBoxLayout>

#include <tobas_qt_tools/widgets/tab_widget.hpp>

namespace tobas::gui::sa::fw::mn
{
namespace
{
constexpr char kVehicleLabel[] = "Vehicle Parameters";
constexpr char kAeroCoefsLabel[] = "Aerodynamic Coefficients";
constexpr char kControlSurfacesLabel[] = "Control Surfaces";
}  // namespace

ManualWidget::ManualWidget(const uadf::Model& _uadf)
{
  vehicle_ = new VehicleWidget();
  aero_coefs_ = new AerodynamicsCoefficientsWidget();
  control_surfaces_ = new ControlSurfacesWidget(_uadf);

  const auto tabs = new qt::TabWidget();
  tabs->setTabWidth(200);
  tabs->enableWheelEvent(false);

  tabs->addTab(vehicle_, kVehicleLabel);
  tabs->addTab(aero_coefs_, kAeroCoefsLabel);
  tabs->addTab(control_surfaces_, kControlSurfacesLabel);

  const auto rows = new QVBoxLayout(this);
  rows->addWidget(tabs);
}

const char* ManualWidget::name() const
{
  return "Manual";
}

void ManualWidget::updateInternalDataStructures()
{
  vehicle_->updateInternalDataStructures();
  aero_coefs_->updateInternalDataStructures();
  control_surfaces_->updateInternalDataStructures();
}

void ManualWidget::setToDefaults()
{
  vehicle_->setToDefaults();
  aero_coefs_->setToDefaults();
  control_surfaces_->setToDefaults();
}

bool ManualWidget::isValid()
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

YAML::Node ManualWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[kVehicleLabel] = vehicle_->dump();
  node[kAeroCoefsLabel] = aero_coefs_->dump();
  node[kControlSurfacesLabel] = control_surfaces_->dump();

  return node;
}

void ManualWidget::load(const YAML::Node& _node)
{
  vehicle_->load(_node[kVehicleLabel]);
  aero_coefs_->load(_node[kAeroCoefsLabel]);
  control_surfaces_->load(_node[kControlSurfacesLabel]);
}

VehicleWidget* ManualWidget::vehicle() const
{
  return vehicle_;
}

AerodynamicsCoefficientsWidget* ManualWidget::aeroCoefs() const
{
  return aero_coefs_;
}

ControlSurfacesWidget* ManualWidget::controlSurfaces() const
{
  return control_surfaces_;
}
}  // namespace tobas::gui::sa::fw::mn
