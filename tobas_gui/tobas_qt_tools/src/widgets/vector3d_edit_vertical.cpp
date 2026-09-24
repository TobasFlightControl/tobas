// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_qt_tools/widgets/vector3d_edit_vertical.hpp"

#include <QFormLayout>

namespace tobas
{
namespace qt
{
Vector3dEditVertical::Vector3dEditVertical(const QStringList& labels, QWidget* parent) : super(parent)
{
  const auto form = new QFormLayout();
  setLayout(form);

  constexpr std::array kDefaultLabels = { "X", "Y", "Z" };
  for (size_t i = 0; i < spin_boxes_.size(); ++i) {
    spin_boxes_[i] = new DoubleSpinBox();
    form->addRow(labels.size() > static_cast<int>(i) ? labels.at(i) : kDefaultLabels[i], spin_boxes_[i]);
    connect(spin_boxes_[i], qOverload<double>(&DoubleSpinBox::valueChanged), this, &self::onValueChanged);
  }
}

Eigen::Vector3d Vector3dEditVertical::vector() const
{
  return { x(), y(), z() };
}

void Vector3dEditVertical::setVector(const Eigen::Vector3d& src)
{
  for (size_t i = 0; i < spin_boxes_.size(); ++i) {
    spin_boxes_[i]->setValue(src[i]);
  }
}

void Vector3dEditVertical::setDecimals(int decimals)
{
  for (const auto spin_box : spin_boxes_) {
    spin_box->setDecimals(decimals);
  }
}

void Vector3dEditVertical::setMinimum(double minimum)
{
  for (const auto spin_box : spin_boxes_) {
    spin_box->setMinimum(minimum);
  }
}

void Vector3dEditVertical::setMaximum(double maximum)
{
  for (const auto spin_box : spin_boxes_) {
    spin_box->setMaximum(maximum);
  }
}

void Vector3dEditVertical::setRange(double minimum, double maximum)
{
  for (const auto spin_box : spin_boxes_) {
    spin_box->setRange(minimum, maximum);
  }
}

void Vector3dEditVertical::setSingleStep(double single_step)
{
  for (const auto spin_box : spin_boxes_) {
    spin_box->setSingleStep(single_step);
  }
}

void Vector3dEditVertical::setSuffix(const QString& suffix)
{
  for (const auto spin_box : spin_boxes_) {
    spin_box->setSuffix(suffix);
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
}  // namespace qt
}  // namespace tobas
