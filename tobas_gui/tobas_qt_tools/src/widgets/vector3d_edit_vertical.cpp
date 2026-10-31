// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_qt_tools/widgets/vector3d_edit_vertical.hpp"

#include <QFormLayout>

namespace tobas::qt
{
Vector3dEditVertical::Vector3dEditVertical(const QStringList& _labels, QWidget* _parent) : super(_parent)
{
  const auto form = new QFormLayout(this);

  constexpr std::array kDefaultLabels = { "X", "Y", "Z" };
  for (size_t i = 0; i < spin_boxes_.size(); ++i) {
    spin_boxes_[i] = new DoubleSpinBox();
    form->addRow(_labels.size() > static_cast<int>(i) ? _labels.at(i) : kDefaultLabels[i], spin_boxes_[i]);
    connect(spin_boxes_[i], qOverload<double>(&DoubleSpinBox::valueChanged), this, &self::onValueChanged);
  }
}

Eigen::Vector3d Vector3dEditVertical::vector() const
{
  return { x(), y(), z() };
}

void Vector3dEditVertical::setVector(const Eigen::Vector3d& _src)
{
  for (size_t i = 0; i < spin_boxes_.size(); ++i) {
    spin_boxes_[i]->setValue(_src[i]);
  }
}

void Vector3dEditVertical::setDecimals(int _decimals)
{
  for (const auto spin_box : spin_boxes_) {
    spin_box->setDecimals(_decimals);
  }
}

void Vector3dEditVertical::setMinimum(double _minimum)
{
  for (const auto spin_box : spin_boxes_) {
    spin_box->setMinimum(_minimum);
  }
}

void Vector3dEditVertical::setMaximum(double _maximum)
{
  for (const auto spin_box : spin_boxes_) {
    spin_box->setMaximum(_maximum);
  }
}

void Vector3dEditVertical::setRange(double _minimum, double _maximum)
{
  for (const auto spin_box : spin_boxes_) {
    spin_box->setRange(_minimum, _maximum);
  }
}

void Vector3dEditVertical::setSingleStep(double _single_step)
{
  for (const auto spin_box : spin_boxes_) {
    spin_box->setSingleStep(_single_step);
  }
}

void Vector3dEditVertical::setSuffix(const QString& _suffix)
{
  for (const auto spin_box : spin_boxes_) {
    spin_box->setSuffix(_suffix);
  }
}

double Vector3dEditVertical::x() const
{
  return spin_boxes_[0]->value();
}

double Vector3dEditVertical::y() const
{
  return spin_boxes_[1]->value();
}

double Vector3dEditVertical::z() const
{
  return spin_boxes_[2]->value();
}

void Vector3dEditVertical::onValueChanged(double)
{
  Q_EMIT valueChanged({ x(), y(), z() });
}
}  // namespace tobas::qt
