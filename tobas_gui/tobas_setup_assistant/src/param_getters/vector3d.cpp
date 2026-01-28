#include "tobas_setup_assistant/param_getters/vector3d.hpp"

#include <tobas_string_tools/core.hpp>

namespace gui
{
namespace sa
{
ParamGetterWidget_Vector3d::ParamGetterWidget_Vector3d(const QString& param_name, const QString& description_text)
  : super(param_name, description_text)
{
  vector3d_ = new qt::Vector3dEditHorizontal();
  rows_->addWidget(vector3d_);
  connect(vector3d_, &qt::Vector3dEditHorizontal::valueChanged, this, &self::onValueChanged);
}

Eigen::Vector3d ParamGetterWidget_Vector3d::getValue() const
{
  return vector3d_->vector();
}

bool ParamGetterWidget_Vector3d::setValue(const Eigen::Vector3d& src)
{
  vector3d_->setVector(src);
  return true;
}

void ParamGetterWidget_Vector3d::setDecimals(int decimals)
{
  vector3d_->setDecimals(decimals);
}

void ParamGetterWidget_Vector3d::setMinimum(double minimum)
{
  vector3d_->setMinimum(minimum);
}

void ParamGetterWidget_Vector3d::setMaximum(double maximum)
{
  vector3d_->setMaximum(maximum);
}

void ParamGetterWidget_Vector3d::setSingleStep(double single_step)
{
  vector3d_->setSingleStep(single_step);
}

void ParamGetterWidget_Vector3d::setSuffix(const QString& suffix)
{
  vector3d_->setSuffix(QString::fromStdString(str::convertToSuperscript(suffix.toStdString())));
}

void ParamGetterWidget_Vector3d::onValueChanged(const Eigen::Vector3d& value)
{
  Q_EMIT valueChanged(value);
}
}  // namespace sa
}  // namespace gui
