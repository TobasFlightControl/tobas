#include "tobas_setup_assistant/setting_tabs/simulation.hpp"

namespace gui
{
namespace setup_assistant
{
const char* SimulationWidget::name() const
{
  return "Simulation";
}

const char* SimulationWidget::title() const
{
  return "Define Simulation Environment";
}

const char* SimulationWidget::description() const
{
  return "Configure the settings for the Gazebo simulation environment. "
         "To enhance the accuracy of the simulation, please input information about the actual operating environment.";
}

void SimulationWidget::onInit()
{
  latitude_zero_ = new ParamGetterWidget_DoubleSpinBox("Latitude of Origin", "");
  latitude_zero_->setDecimals(6);
  latitude_zero_->setMinimum(-90.0);
  latitude_zero_->setMaximum(+90.0);
  latitude_zero_->setValue(35.658099);  // 日本: 北緯35度39分29秒
  latitude_zero_->setSuffix(" deg");
  addWidget(latitude_zero_);

  longitude_zero_ = new ParamGetterWidget_DoubleSpinBox("Longitude of Origin", "");
  longitude_zero_->setDecimals(6);
  longitude_zero_->setMinimum(-180.0);
  longitude_zero_->setMaximum(+180.0);
  longitude_zero_->setValue(139.741354);  // 日本: 東経139度44分28秒8759
  longitude_zero_->setSuffix(" deg");
  addWidget(longitude_zero_);

  altitude_zero_ = new ParamGetterWidget_DoubleSpinBox("Altitude Above Sea Level", "");
  altitude_zero_->setDecimals(3);
  altitude_zero_->setValue(24.39);  // 日本水準原点: https://www.gsi.go.jp/sokuchikijun/suijun-base.html
  altitude_zero_->setSuffix(" m");
  addWidget(altitude_zero_);

  max_model_error_rate_ = new ParamGetterWidget_SpinBox("Max Model Error Rate", "");
  max_model_error_rate_->setMinimum(0);
  max_model_error_rate_->setMaximum(100);
  max_model_error_rate_->setValue(10);
  max_model_error_rate_->setSuffix(" %");
  addWidget(max_model_error_rate_);

  addStretch();
}

void SimulationWidget::onOpened()
{
  return;
}

void SimulationWidget::updateInternalDataStructures()
{
  return;
}

bool SimulationWidget::isValid()
{
  // TODO: 極に近すぎると方角がわからない
  return true;
}

YAML::Node SimulationWidget::dump()
{
  YAML::Node node(YAML::NodeType::Map);

  node[latitude_zero_->name()] = latitude_zero_->getValue();
  node[longitude_zero_->name()] = longitude_zero_->getValue();
  node[altitude_zero_->name()] = altitude_zero_->getValue();
  node[max_model_error_rate_->name()] = max_model_error_rate_->getValue();

  return node;
}

void SimulationWidget::load(const YAML::Node& node)
{
  latitude_zero_->setValue(node[latitude_zero_->name()].as<double>());
  longitude_zero_->setValue(node[longitude_zero_->name()].as<double>());
  altitude_zero_->setValue(node[altitude_zero_->name()].as<double>());
  max_model_error_rate_->setValue(node[max_model_error_rate_->name()].as<double>());
}

double SimulationWidget::latitudeZero() const
{
  return latitude_zero_->getValue();
}

double SimulationWidget::longitudeZero() const
{
  return longitude_zero_->getValue();
}

double SimulationWidget::altitudeZero() const
{
  return altitude_zero_->getValue();
}

double SimulationWidget::maxModelErrorRate() const
{
  return static_cast<double>(max_model_error_rate_->getValue()) / 100.;
}
}  // namespace setup_assistant
}  // namespace gui
