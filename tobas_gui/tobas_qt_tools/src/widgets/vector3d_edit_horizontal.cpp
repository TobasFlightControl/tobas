#include "tobas_qt_tools/widgets/vector3d_edit_horizontal.hpp"

#include <QHBoxLayout>

namespace tobas
{
namespace qt
{
Vector3dEditHorizontal::Vector3dEditHorizontal(QWidget* parent) : super(parent)
{
  const auto cols = new QHBoxLayout();
  setLayout(cols);

  x_ = new LabeledDoubleSpinBox("X");
  cols->addWidget(x_);

  y_ = new LabeledDoubleSpinBox("Y");
  cols->addWidget(y_);

  z_ = new LabeledDoubleSpinBox("Z");
  cols->addWidget(z_);

  connect(x_, &LabeledDoubleSpinBox::valueChanged, this, &self::onValueChanged);
  connect(y_, &LabeledDoubleSpinBox::valueChanged, this, &self::onValueChanged);
  connect(z_, &LabeledDoubleSpinBox::valueChanged, this, &self::onValueChanged);
}

Eigen::Vector3d Vector3dEditHorizontal::vector() const
{
  return { x(), y(), z() };
}

void Vector3dEditHorizontal::setVector(const Eigen::Vector3d& src)
{
  x_->setValue(src.x());
  y_->setValue(src.y());
  z_->setValue(src.z());
}

void Vector3dEditHorizontal::setDecimals(int decimals)
{
  x_->setDecimals(decimals);
  y_->setDecimals(decimals);
  z_->setDecimals(decimals);
}

void Vector3dEditHorizontal::setMinimum(double minimum)
{
  x_->setMinimum(minimum);
  y_->setMinimum(minimum);
  z_->setMinimum(minimum);
}

void Vector3dEditHorizontal::setMaximum(double maximum)
{
  x_->setMaximum(maximum);
  y_->setMaximum(maximum);
  z_->setMaximum(maximum);
}

void Vector3dEditHorizontal::setSingleStep(double single_step)
{
  x_->setSingleStep(single_step);
  y_->setSingleStep(single_step);
  z_->setSingleStep(single_step);
}

void Vector3dEditHorizontal::setSuffix(const QString& suffix)
{
  x_->setSuffix(suffix);
  y_->setSuffix(suffix);
  z_->setSuffix(suffix);
}

double Vector3dEditHorizontal::x() const
{
  return x_->getValue();
}

double Vector3dEditHorizontal::y() const
{
  return y_->getValue();
}

double Vector3dEditHorizontal::z() const
{
  return z_->getValue();
}

void Vector3dEditHorizontal::onValueChanged(double)
{
  Q_EMIT valueChanged({ x(), y(), z() });
}
}  // namespace qt
}  // namespace tobas
