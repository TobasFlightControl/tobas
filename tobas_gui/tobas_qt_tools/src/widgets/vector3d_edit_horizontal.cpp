// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_qt_tools/widgets/vector3d_edit_horizontal.hpp"

#include <QHBoxLayout>

namespace tobas
{
namespace qt
{
Vector3dEditHorizontal::Vector3dEditHorizontal(QWidget* _parent) : super(_parent)
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

void Vector3dEditHorizontal::setVector(const Eigen::Vector3d& _src)
{
  x_->setValue(_src.x());
  y_->setValue(_src.y());
  z_->setValue(_src.z());
}

void Vector3dEditHorizontal::setDecimals(int _decimals)
{
  x_->setDecimals(_decimals);
  y_->setDecimals(_decimals);
  z_->setDecimals(_decimals);
}

void Vector3dEditHorizontal::setMinimum(double _minimum)
{
  x_->setMinimum(_minimum);
  y_->setMinimum(_minimum);
  z_->setMinimum(_minimum);
}

void Vector3dEditHorizontal::setMaximum(double _maximum)
{
  x_->setMaximum(_maximum);
  y_->setMaximum(_maximum);
  z_->setMaximum(_maximum);
}

void Vector3dEditHorizontal::setSingleStep(double _single_step)
{
  x_->setSingleStep(_single_step);
  y_->setSingleStep(_single_step);
  z_->setSingleStep(_single_step);
}

void Vector3dEditHorizontal::setSuffix(const QString& _suffix)
{
  x_->setSuffix(_suffix);
  y_->setSuffix(_suffix);
  z_->setSuffix(_suffix);
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
