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
FixedWingWidget::FixedWingWidget(const uadf::Model& uadf, const kdl::Tree& tree)
{
  // Vehicle
  addWidget(new qt::Label(kVehicleLabel, cmn::kTitlePSize));
  vehicle_ = new VehicleParametersWidget();
  addWidget(vehicle_);

  tabs_ = new qt::TabWidget();
  tabs_->enableWheelEvent(false);
  tabs_->setTabSize(kTabWidth, kTabHeight);
  addWidget(tabs_);

  // coefficients widget
  coefs_ = new CoefficientsWidget(uadf);
  tabs_->addTab(coefs_, coefs_->name());

  // helper widget
  helper_ = new hp::HelperWidget(uadf, tree, vehicle_, coefs_);
  tabs_->addTab(helper_, helper_->name());

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
  coefs_->updateInternalDataStructures();
  helper_->updateInternalDataStructures();
}

void FixedWingWidget::setToDefaults()
{
  vehicle_->setToDefaults();
  coefs_->setToDefaults();
  helper_->setToDefaults();
}

bool FixedWingWidget::isValid()
{
  if (!vehicle_->isValid()) {
    return false;
  }
  if (!coefs_->isValid()) {
    return false;
  }
  if (helper_->isValid()) {
    return false;
  }

  return true;
}

YAML::Node FixedWingWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[kVehicleLabel] = vehicle_->dump();
  for (const auto& item: coefs_->dump()) {
    node[item.first] = item.second;
  }

  return node;
}

void FixedWingWidget::load(const YAML::Node& node)
{
  vehicle_->load(node[kVehicleLabel]);
  coefs_->load(node);
}

const VehicleParametersWidget* FixedWingWidget::vehicle() const
{
  return vehicle_;
}

const AerodynamicsCoefficientsWidget* FixedWingWidget::aeroCoefs() const
{
  return coefs_->aeroCoefs();
}

const ControlSurfacesWidget* FixedWingWidget::controlSurfaces() const
{
  return coefs_->controlSurfaces();
}
}  // namespace fw
}  // namespace sa
}  // namespace gui
}  // namespace tobas
