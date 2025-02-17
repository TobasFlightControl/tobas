#include "tobas_setup_assistant/param_getters/vector3d.hpp"

namespace gui
{
namespace sa
{
ParamGetterWidget_Vector3d::ParamGetterWidget_Vector3d(const QString& param_name, const QString& description_text)
  : super(param_name, description_text)
{
  const auto cols = new QHBoxLayout();
  rows_->addLayout(cols);

  x_ = new DoubleGetter("x");
  cols->addWidget(x_);

  y_ = new DoubleGetter("y");
  cols->addWidget(y_);

  z_ = new DoubleGetter("z");
  cols->addWidget(z_);

  connect(x_, QOverload<double>::of(&DoubleGetter::valueChanged), this, &self::onValueChanged);
  connect(y_, QOverload<double>::of(&DoubleGetter::valueChanged), this, &self::onValueChanged);
  connect(z_, QOverload<double>::of(&DoubleGetter::valueChanged), this, &self::onValueChanged);
}

Eigen::Vector3d ParamGetterWidget_Vector3d::getValue() const
{
  return { x(), y(), z() };
}

bool ParamGetterWidget_Vector3d::setValue(const Eigen::Vector3d& src)
{
  x_->setValue(src.x());
  y_->setValue(src.y());
  z_->setValue(src.z());
  return true;
}

void ParamGetterWidget_Vector3d::setDecimals(int decimals)
{
  x_->setDecimals(decimals);
  y_->setDecimals(decimals);
  z_->setDecimals(decimals);
}

void ParamGetterWidget_Vector3d::setMinimum(double minimum)
{
  x_->setMinimum(minimum);
  y_->setMinimum(minimum);
  z_->setMinimum(minimum);
}

void ParamGetterWidget_Vector3d::setMaximum(double maximum)
{
  x_->setMaximum(maximum);
  y_->setMaximum(maximum);
  z_->setMaximum(maximum);
}

void ParamGetterWidget_Vector3d::setSingleStep(double single_step)
{
  x_->setSingleStep(single_step);
  y_->setSingleStep(single_step);
  z_->setSingleStep(single_step);
}

void ParamGetterWidget_Vector3d::setSuffix(const QString& suffix)
{
  x_->setSuffix(suffix);
  y_->setSuffix(suffix);
  z_->setSuffix(suffix);
}

double ParamGetterWidget_Vector3d::x() const
{
  return x_->getValue();
}

double ParamGetterWidget_Vector3d::y() const
{
  return y_->getValue();
}

double ParamGetterWidget_Vector3d::z() const
{
  return z_->getValue();
}

void ParamGetterWidget_Vector3d::onValueChanged(double)
{
  Q_EMIT valueChanged({ x(), y(), z() });
}
}  // namespace sa
}  // namespace gui
