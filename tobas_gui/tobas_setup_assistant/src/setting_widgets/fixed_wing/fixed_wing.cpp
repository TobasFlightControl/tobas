// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/setting_tabs/fixed_wing/fixed_wing.hpp"

#include <QVBoxLayout>

#include <tobas_gui_common/constants.hpp>
#include <tobas_qt_tools/widgets/label.hpp>

namespace tobas
{
namespace gui
{
namespace sa
{
namespace fw
{
FixedWingWidget::FixedWingWidget(const uadf::Model& uadf)
{
  // Vehicle
  addWidget(new qt::Label(kVehicleLabel, cmn::kTitlePSize));
  vehicle_ = new VehicleParametersWidget();
  addWidget(vehicle_);

  // Aerodynamic Coefficients
  addWidget(new qt::Label(kAeroCoefsLabel, cmn::kTitlePSize));
  aero_coefs_ = new AerodynamicsCoefficientsWidget();
  addWidget(aero_coefs_);

  // Control Surfaces
  addWidget(new qt::Label(kControlSurfacesLabel, cmn::kTitlePSize));
  control_surfaces_ = new ControlSurfacesWidget(uadf);
  addWidget(control_surfaces_);
}

const char* FixedWingWidget::name() const
{
  return "Fixed Wing";
}

const char* FixedWingWidget::title() const
{
  return "Define Fixed Wing";
}

const char* FixedWingWidget::description() const
{
  return "Build the mathematical model for the fixed wing and its control surfaces. "
         "In addition to the general airframe specifications, "
         "supply the stability derivatives for the main wing and each control surface. "
         "<a href="
         "'https://vspu.larc.nasa.gov/training-content/chapter-3-model-analysis-in-openvsp/vspaero-basics'"
         ">VSPAERO</a> "
         "analysis results can be imported for the main wing if available.";
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

void FixedWingWidget::load(const YAML::Node& node)
{
  vehicle_->load(node[kVehicleLabel]);
  aero_coefs_->load(node[kAeroCoefsLabel]);
  control_surfaces_->load(node[kControlSurfacesLabel]);
}

const VehicleParametersWidget* FixedWingWidget::vehicle() const
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
}  // namespace fw
}  // namespace sa
}  // namespace gui
}  // namespace tobas
