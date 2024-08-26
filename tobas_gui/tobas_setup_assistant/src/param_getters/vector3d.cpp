#include "tobas_setup_assistant/param_getters/vector3d.hpp"

namespace gui
{
namespace setup_assistant
{
ParamGetterWidget_Vector3d::ParamGetterWidget_Vector3d(
  const QString& param_name,
  const QString& description_text,
  int decimals,
  double minimum,
  double maximum,
  double single_step,
  const Eigen::Vector3d& _default,
  const QString& suffix)
  : super(param_name, description_text)
{
  auto cols = new QHBoxLayout();
  rows_->addLayout(cols);

  x_ = new DoubleGetter("x", decimals, minimum, maximum, single_step, _default.x(), suffix);
  cols->addWidget(x_);

  y_ = new DoubleGetter("y", decimals, minimum, maximum, single_step, _default.y(), suffix);
  cols->addWidget(y_);

  z_ = new DoubleGetter("z", decimals, minimum, maximum, single_step, _default.z(), suffix);
  cols->addWidget(z_);

  connect(x_, SIGNAL(IntGetter::valueChanged(double)), this, SLOT(onValueChanged(double)));
  connect(y_, SIGNAL(IntGetter::valueChanged(double)), this, SLOT(onValueChanged(double)));
  connect(z_, SIGNAL(IntGetter::valueChanged(double)), this, SLOT(onValueChanged(double)));
}

Eigen::Vector3d ParamGetterWidget_Vector3d::get() const
{
  return { x(), y(), z() };
}

bool ParamGetterWidget_Vector3d::set(const Eigen::Vector3d& src)
{
  x_->set(src.x());
  y_->set(src.y());
  z_->set(src.z());
  return true;
}

double ParamGetterWidget_Vector3d::x() const
{
  return x_->get();
}

double ParamGetterWidget_Vector3d::y() const
{
  return y_->get();
}

double ParamGetterWidget_Vector3d::z() const
{
  return z_->get();
}

void ParamGetterWidget_Vector3d::onValueChanged(double)
{
  Q_EMIT valueChanged({ x(), y(), z() });
}
}  // namespace setup_assistant
}  // namespace gui
