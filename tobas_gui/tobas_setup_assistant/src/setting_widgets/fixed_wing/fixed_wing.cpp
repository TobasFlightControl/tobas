#include "tobas_setup_assistant/setting_tabs/fixed_wing/fixed_wing.hpp"

#include <tobas_qt_tools/widgets/label.hpp>

namespace gui
{
namespace sa
{
namespace fixed_wing
{
FixedWingWidget::FixedWingWidget(rclcpp::Node::SharedPtr node, const RobotInfo& robot) : node_(node), robot_(robot)
{
  has_fixed_wing_ = new QCheckBox("Fixed-Wing Configuration");
  has_fixed_wing_->setFont(qt::DefaultFont(kBodyPSize));
  has_fixed_wing_->setChecked(kDefaultHasFixedWing);
  connect(has_fixed_wing_, &QCheckBox::toggled, this, &self::setSettingWidgetsEnabled);
  addWidget(has_fixed_wing_);

  addSpacing(50);

  setting_rows_ = new QVBoxLayout();
  addLayout(setting_rows_);

  // Vehicle
  setting_rows_->addWidget(new qt::Label(kVehicleLabel, kTitlePSize));
  vehicle_ = new VehicleParametersWidget();
  setting_rows_->addWidget(vehicle_);

  // Aerodynamic Coefficients
  setting_rows_->addWidget(new qt::Label(kAeroCoefsLabel, kTitlePSize));
  aero_coefs_ = new AerodynamicsCoefficientsWidget(node_);
  setting_rows_->addWidget(aero_coefs_);

  // Control Surfaces
  setting_rows_->addWidget(new qt::Label(kControlSurfacesLabel, kTitlePSize));
  control_surfaces_ = new ControlSurfacesWidget(robot_);
  setting_rows_->addWidget(control_surfaces_);

  setSettingWidgetsEnabled(kDefaultHasFixedWing);
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

void FixedWingWidget::onOpened()
{
}

void FixedWingWidget::updateInternalDataStructures()
{
  vehicle_->updateInternalDataStructures();
  aero_coefs_->updateInternalDataStructures();
  control_surfaces_->updateInternalDataStructures();
}

bool FixedWingWidget::isValid()
{
  if (!hasFixedWing()) {
    return true;
  }

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

  node[has_fixed_wing_->text().toStdString()] = has_fixed_wing_->isChecked();

  node[kVehicleLabel] = vehicle_->dump();
  node[kAeroCoefsLabel] = aero_coefs_->dump();
  node[kControlSurfacesLabel] = control_surfaces_->dump();

  return node;
}

void FixedWingWidget::load(const YAML::Node& node)
{
  has_fixed_wing_->setChecked(node[has_fixed_wing_->text().toStdString()].as<bool>());

  vehicle_->load(node[kVehicleLabel]);
  aero_coefs_->load(node[kAeroCoefsLabel]);
  control_surfaces_->load(node[kControlSurfacesLabel]);
}

bool FixedWingWidget::hasFixedWing() const
{
  return has_fixed_wing_->isChecked();
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

void FixedWingWidget::setSettingWidgetsEnabled(bool enabled)
{
  for (int row = 0; row < setting_rows_->count(); ++row) {
    const auto widget = setting_rows_->itemAt(row)->widget();
    widget->setEnabled(enabled);
  }
}
}  // namespace fixed_wing
}  // namespace sa
}  // namespace gui
