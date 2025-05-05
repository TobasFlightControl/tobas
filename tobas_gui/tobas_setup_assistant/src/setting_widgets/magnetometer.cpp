#include "tobas_setup_assistant/setting_tabs/magnetometer.hpp"

#include <tobas_yaml_tools/convert/eigen.hpp>

namespace gui
{
namespace sa
{
MagnetometerWidget::MagnetometerWidget()
{
  offset_ = new ParamGetterWidget_Vector3d("Offset", kSensorOffsetDescription);
  offset_->setSuffix(" m");
  addWidget(offset_);

  update_rate_ = new ParamGetterWidget_SpinBox("Update Rate", "");  // TODO
  update_rate_->setMinimum(1);
  update_rate_->setValue(100);
  update_rate_->setSuffix(" Hz");
  addWidget(update_rate_);

  noise_stddev_ = new ParamGetterWidget_SpinBox("Standard Deviation of Additive White Gaussian Noise", "");  // TODO
  noise_stddev_->setMinimum(0);
  noise_stddev_->setValue(460);  // IIS2MDC
  noise_stddev_->setSuffix(" nT");
  addWidget(noise_stddev_);

  hard_bias_norm_ = new ParamGetterWidget_SpinBox("Hard-Iron Bias Norm", "");  // TODO
  hard_bias_norm_->setMinimum(0);
  hard_bias_norm_->setValue(3000);  // IIS2MDCの最大値は6000nTだが，キャリブレーションを前提としてそれより低めに設定．
  hard_bias_norm_->setSuffix(" nT");
  addWidget(hard_bias_norm_);

  addStretch();
}

const char* MagnetometerWidget::name() const
{
  return "Compass";
}

const char* MagnetometerWidget::title() const
{
  return "Define 3-Axis Magnetometer";
}

const char* MagnetometerWidget::description() const
{
  return "";  // TODO
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

YAML::Node MagnetometerWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[offset_->name()] = offset_->getValue();
  node[update_rate_->name()] = update_rate_->getValue();
  node[noise_stddev_->name()] = noise_stddev_->getValue();
  node[hard_bias_norm_->name()] = hard_bias_norm_->getValue();

  return node;
}

void MagnetometerWidget::load(const YAML::Node& node)
{
  offset_->setValue(node[offset_->name()].as<Eigen::Vector3d>());
  update_rate_->setValue(node[update_rate_->name()].as<int>());
  noise_stddev_->setValue(node[noise_stddev_->name()].as<int>());
  hard_bias_norm_->setValue(node[hard_bias_norm_->name()].as<int>());
}

Eigen::Vector3d MagnetometerWidget::offset() const
{
  return offset_->getValue();
}

int MagnetometerWidget::updateRate() const
{
  return update_rate_->getValue();
}

int MagnetometerWidget::noiseStddev() const
{
  return noise_stddev_->getValue();
}

int MagnetometerWidget::hardBiasNorm() const
{
  return hard_bias_norm_->getValue();
}
}  // namespace sa
}  // namespace gui
