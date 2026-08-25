#include <tobas_setup_assistant/setting_tabs/fixed_wing/coefs/coefs.hpp>

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
namespace cf
{
CoefficientsWidget::CoefficientsWidget(const uadf::Model& uadf)
{
  const auto rows = new QVBoxLayout();
  setLayout(rows);

  // Vehicle Parameters
  rows->addWidget(new qt::Label(kVehicleLabel, cmn::kTitlePSize));
  vehicle_ = new VehicleParametersWidget();
  rows->addWidget(vehicle_);

  // Aerodynamic Coefficients
  rows->addWidget(new qt::Label(kAeroCoefsLabel, cmn::kTitlePSize));
  aero_coefs_ = new AerodynamicsCoefficientsWidget();
  rows->addWidget(aero_coefs_);

  // Control Surfaces
  rows->addWidget(new qt::Label(kControlSurfacesLabel, cmn::kTitlePSize));
  control_surfaces_ = new ControlSurfacesWidget(uadf);
  rows->addWidget(control_surfaces_);
}

const char* CoefficientsWidget::name() const
{
  return "Coefficients";
}

void CoefficientsWidget::updateInternalDataStructures()
{
  vehicle_->updateInternalDataStructures();
  aero_coefs_->updateInternalDataStructures();
  control_surfaces_->updateInternalDataStructures();
}

void CoefficientsWidget::setToDefaults()
{
  vehicle_->setToDefaults();
  aero_coefs_->setToDefaults();
  control_surfaces_->setToDefaults();
}

bool CoefficientsWidget::isValid()
{
  return vehicle_->isValid() && aero_coefs_->isValid() && control_surfaces_->isValid();
}

YAML::Node CoefficientsWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[kVehicleLabel] = vehicle_->dump();
  node[kAeroCoefsLabel] = aero_coefs_->dump();
  node[kControlSurfacesLabel] = control_surfaces_->dump();

  return node;
}

void CoefficientsWidget::load(const YAML::Node& node)
{
  vehicle_->load(node[kVehicleLabel]);
  aero_coefs_->load(node[kAeroCoefsLabel]);
  control_surfaces_->load(node[kControlSurfacesLabel]);
}

VehicleParametersWidget* CoefficientsWidget::vehicle() const
{
  return vehicle_;
}

AerodynamicsCoefficientsWidget* CoefficientsWidget::aeroCoefs() const
{
  return aero_coefs_;
}

ControlSurfacesWidget* CoefficientsWidget::controlSurfaces() const
{
  return control_surfaces_;
}
}  // namespace cf
}  // namespace fw
}  // namespace sa
}  // namespace gui
}  // namespace tobas
