#include <tobas_yaml_tools/convert/eigen.hpp>
#include <tobas_qt_tools/message.hpp>

#include "tobas_setup_assistant/setting_tabs/fixed_wing/vehicle.hpp"

namespace gui
{
namespace sa
{
namespace fixed_wing
{
VehicleParametersWidget::VehicleParametersWidget()
{
  const auto rows = new QVBoxLayout();
  setLayout(rows);

  wing_surface_ = new ParamGetterWidget_DoubleSpinBox("Wing Surface", "");
  wing_surface_->setDecimals(3);
  wing_surface_->setMinimum(1e-3);
  wing_surface_->setValue(0.47);
  wing_surface_->setSuffix(" m^2");
  rows->addWidget(wing_surface_);

  wing_span_ = new ParamGetterWidget_DoubleSpinBox("Wing Span", "");
  wing_span_->setDecimals(3);
  wing_span_->setMinimum(1e-3);
  wing_span_->setValue(2.59);
  wing_span_->setSuffix(" m");
  rows->addWidget(wing_span_);

  mac_ = new ParamGetterWidget_DoubleSpinBox("Mean Aerodynamic Chord", "");
  mac_->setDecimals(3);
  mac_->setMinimum(1e-3);
  mac_->setValue(0.18);
  mac_->setSuffix(" m");
  rows->addWidget(mac_);

  aerodynamic_center_ = new ParamGetterWidget_Vector3d("Aerodynamic Center", "");
  aerodynamic_center_->setDecimals(3);
  aerodynamic_center_->setValue({ 0.1, 0., 0. });
  aerodynamic_center_->setSuffix(" m");
  rows->addWidget(aerodynamic_center_);

  alpha_limit_ = new ParamGetterWidget_DoubleRange("Limitation of Angle of Attack", "");
  alpha_limit_->setDecimals(3);
  alpha_limit_->setValue({ -0.27, 0.27 });
  alpha_limit_->setSuffix(" rad");
  rows->addWidget(alpha_limit_);
}

void VehicleParametersWidget::updateInternalDataStructures()
{
}

bool VehicleParametersWidget::isValid()
{
  if (!alpha_limit_->isValid())
  {
    qt::qErrorBox(this, "Invalid limitation of angle of attack.");
    return false;
  }

  return true;
}

YAML::Node VehicleParametersWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[wing_surface_->name()] = wing_surface_->getValue();
  node[wing_span_->name()] = wing_span_->getValue();
  node[mac_->name()] = mac_->getValue();
  node[aerodynamic_center_->name()] = aerodynamic_center_->getValue();
  node[alpha_limit_->name()] = alpha_limit_->getValue();

  return node;
}

void VehicleParametersWidget::load(const YAML::Node& node)
{
  wing_surface_->setValue(node[wing_surface_->name()].as<double>());
  wing_span_->setValue(node[wing_span_->name()].as<double>());
  mac_->setValue(node[mac_->name()].as<double>());
  aerodynamic_center_->setValue(node[aerodynamic_center_->name()].as<Eigen::Vector3d>());
  alpha_limit_->setValue(node[alpha_limit_->name()].as<std::pair<double, double>>());
}

double VehicleParametersWidget::wingSurface() const
{
  return wing_surface_->getValue();
}

double VehicleParametersWidget::wingSpan() const
{
  return wing_span_->getValue();
}

double VehicleParametersWidget::mac() const
{
  return mac_->getValue();
}

Eigen::Vector3d VehicleParametersWidget::aerodynamicCenter() const
{
  return aerodynamic_center_->getValue();
}

std::pair<double, double> VehicleParametersWidget::alphaLimit() const
{
  return alpha_limit_->getValue();
}
}  // namespace fixed_wing
}  // namespace sa
}  // namespace gui
