#include "tobas_setup_assistant/setting_tabs/fixed_wing/fixed_wing.hpp"

#include <QVBoxLayout>

#include <tobas_qt_tools/widgets/label.hpp>

namespace gui
{
namespace sa
{
namespace fw
{
FixedWingWidget::FixedWingWidget(rclcpp::Node::SharedPtr node, const uadf::Model& uadf) : node_(node)
{
  // Vehicle
  addWidget(new qt::Label(kVehicleLabel, kTitlePSize));
  vehicle_ = new VehicleParametersWidget();
  addWidget(vehicle_);

  // Aerodynamic Coefficients
  addWidget(new qt::Label(kAeroCoefsLabel, kTitlePSize));
  aero_coefs_ = new AerodynamicsCoefficientsWidget(node_);
  addWidget(aero_coefs_);

  // Control Surfaces
  addWidget(new qt::Label(kControlSurfacesLabel, kTitlePSize));
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
  return "Set up the fixed-wing configuration. "
         "Please choose a setup method and enter the required information.";
}

void FixedWingWidget::updateInternalDataStructures()
{
  vehicle_->updateInternalDataStructures();
  aero_coefs_->updateInternalDataStructures();
  control_surfaces_->updateInternalDataStructures();
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
