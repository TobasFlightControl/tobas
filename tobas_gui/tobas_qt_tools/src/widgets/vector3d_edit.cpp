#include "tobas_qt_tools/widgets/vector3d_edit.hpp"

#include <QHBoxLayout>

namespace qt
{
Vector3dEdit::Vector3dEdit(QWidget* parent) : super(parent)
{
  const auto cols = new QHBoxLayout();
  setLayout(cols);

  x_ = new LabeledDoubleSpinBox("x");
  cols->addWidget(x_);

  y_ = new LabeledDoubleSpinBox("y");
  cols->addWidget(y_);

  z_ = new LabeledDoubleSpinBox("z");
  cols->addWidget(z_);

  connect(x_, QOverload<double>::of(&LabeledDoubleSpinBox::valueChanged), this, &self::onValueChanged);
  connect(y_, QOverload<double>::of(&LabeledDoubleSpinBox::valueChanged), this, &self::onValueChanged);
  connect(z_, QOverload<double>::of(&LabeledDoubleSpinBox::valueChanged), this, &self::onValueChanged);
}

Eigen::Vector3d Vector3dEdit::vector() const
{
  return { x(), y(), z() };
}

void Vector3dEdit::setVector(const Eigen::Vector3d& src)
{
  x_->setValue(src.x());
  y_->setValue(src.y());
  z_->setValue(src.z());
}

void Vector3dEdit::setDecimals(int decimals)
{
  x_->setDecimals(decimals);
  y_->setDecimals(decimals);
  z_->setDecimals(decimals);
}

void Vector3dEdit::setMinimum(double minimum)
{
  x_->setMinimum(minimum);
  y_->setMinimum(minimum);
  z_->setMinimum(minimum);
}

void Vector3dEdit::setMaximum(double maximum)
{
  x_->setMaximum(maximum);
  y_->setMaximum(maximum);
  z_->setMaximum(maximum);
}

void Vector3dEdit::setSingleStep(double single_step)
{
  x_->setSingleStep(single_step);
  y_->setSingleStep(single_step);
  z_->setSingleStep(single_step);
}

void Vector3dEdit::setSuffix(const QString& suffix)
{
  x_->setSuffix(suffix);
  y_->setSuffix(suffix);
  z_->setSuffix(suffix);
}

double Vector3dEdit::x() const
{
  return x_->getValue();
}

double Vector3dEdit::y() const
{
  return y_->getValue();
}

double Vector3dEdit::z() const
{
  return z_->getValue();
}

void Vector3dEdit::onValueChanged(double)
{
  Q_EMIT valueChanged({ x(), y(), z() });
}
}  // namespace qt
