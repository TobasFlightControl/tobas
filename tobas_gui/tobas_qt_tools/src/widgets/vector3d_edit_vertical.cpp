// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_qt_tools/widgets/vector3d_edit_vertical.hpp"

#include <QFormLayout>

namespace tobas
{
namespace qt
{
Vector3dEditVertical::Vector3dEditVertical(QWidget* parent) : super(parent)
{
  const auto form = new QFormLayout();
  setLayout(form);

  x_ = new DoubleSpinBox();
  form->addRow("X", x_);

  y_ = new DoubleSpinBox();
  form->addRow("Y", y_);

  z_ = new DoubleSpinBox();
  form->addRow("Z", z_);

  connect(x_, QOverload<double>::of(&DoubleSpinBox::valueChanged), this, &self::onValueChanged);
  connect(y_, QOverload<double>::of(&DoubleSpinBox::valueChanged), this, &self::onValueChanged);
  connect(z_, QOverload<double>::of(&DoubleSpinBox::valueChanged), this, &self::onValueChanged);
}

Eigen::Vector3d Vector3dEditVertical::vector() const
{
  return { x(), y(), z() };
}

void Vector3dEditVertical::setVector(const Eigen::Vector3d& src)
{
  x_->setValue(src.x());
  y_->setValue(src.y());
  z_->setValue(src.z());
}

void Vector3dEditVertical::setDecimals(int decimals)
{
  x_->setDecimals(decimals);
  y_->setDecimals(decimals);
  z_->setDecimals(decimals);
}

void Vector3dEditVertical::setMinimum(double minimum)
{
  x_->setMinimum(minimum);
  y_->setMinimum(minimum);
  z_->setMinimum(minimum);
}

void Vector3dEditVertical::setMaximum(double maximum)
{
  x_->setMaximum(maximum);
  y_->setMaximum(maximum);
  z_->setMaximum(maximum);
}

void Vector3dEditVertical::setSingleStep(double single_step)
{
  x_->setSingleStep(single_step);
  y_->setSingleStep(single_step);
  z_->setSingleStep(single_step);
}

void Vector3dEditVertical::setSuffix(const QString& suffix)
{
  x_->setSuffix(suffix);
  y_->setSuffix(suffix);
  z_->setSuffix(suffix);
}

double Vector3dEditVertical::x() const
{
  return x_->value();
}

double Vector3dEditVertical::y() const
{
  return y_->value();
}

double Vector3dEditVertical::z() const
{
  return z_->value();
}

void Vector3dEditVertical::onValueChanged(double)
{
  Q_EMIT valueChanged({ x(), y(), z() });
}
}  // namespace qt
}  // namespace tobas
