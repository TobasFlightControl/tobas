#include <tobas_yaml_tools/convert/eigen.hpp>

#include "tobas_setup_assistant/setting_tabs/magnetometer.hpp"

namespace gui
{
namespace setup_assistant
{
const char* MagnetometerWidget::name() const
{
  return "Compass";
}

const char* MagnetometerWidget::title() const
{
  return "Define Compass";
}

const char* MagnetometerWidget::description() const
{
  return "";  // TODO
}

void MagnetometerWidget::onInit()
{
  offset_ = new ParamGetterWidget_Vector3d("Offset", kSensorOffsetDescription);
  offset_->setSuffix(" m");
  addWidget(offset_);

  update_rate_ = new ParamGetterWidget_SpinBox("Update Rate", "");  // TODO
  update_rate_->setMinimum(1);
  update_rate_->setValue(400);
  update_rate_->setSuffix(" Hz");
  addWidget(update_rate_);

  gauss_noise_ = new ParamGetterWidget_SpinBox("Standard Deviation of Additive White Gaussian Noise", "");  // TODO
  gauss_noise_->setMinimum(0);
  gauss_noise_->setValue(80);
  gauss_noise_->setSuffix(" nT");
  addWidget(gauss_noise_);

  uniform_noise_ = new ParamGetterWidget_SpinBox("Symmetric Bounds of Uniform Noise for Initial Bias", "");  // TODO
  uniform_noise_->setMinimum(0);
  uniform_noise_->setValue(400);
  uniform_noise_->setSuffix(" nT");
  addWidget(uniform_noise_);
}

void MagnetometerWidget::onOpened()
{
  return;
}

void MagnetometerWidget::updateInternalDataStructures()
{
  return;
}

bool MagnetometerWidget::isValid()
{
  return true;
}

YAML::Node MagnetometerWidget::dump()
{
  YAML::Node node(YAML::NodeType::Map);

  node[offset_->name()] = offset_->getValue();
  node[update_rate_->name()] = update_rate_->getValue();
  node[gauss_noise_->name()] = gauss_noise_->getValue();
  node[uniform_noise_->name()] = uniform_noise_->getValue();

  return node;
}

void MagnetometerWidget::load(const YAML::Node& node)
{
  offset_->setValue(node[offset_->name()].as<Eigen::Vector3d>());
  update_rate_->setValue(node[update_rate_->name()].as<int>());
  gauss_noise_->setValue(node[gauss_noise_->name()].as<int>());
  uniform_noise_->setValue(node[uniform_noise_->name()].as<int>());
}

Eigen::Vector3d MagnetometerWidget::offset() const
{
  return offset_->getValue();
}

int MagnetometerWidget::updateRate() const
{
  return update_rate_->getValue();
}

int MagnetometerWidget::gaussNoise() const
{
  return gauss_noise_->getValue();
}

int MagnetometerWidget::uniformNoise() const
{
  return uniform_noise_->getValue();
}
}  // namespace setup_assistant
}  // namespace gui
