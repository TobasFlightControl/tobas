// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QWidget>
#include <eigen3/Eigen/Core>

#include "./labeled_double_spin_box.hpp"

namespace tobas
{
namespace qt
{
class Vector3dEditHorizontal : public QWidget
{
  Q_OBJECT

  using self = Vector3dEditHorizontal;
  using super = QWidget;

Q_SIGNALS:
  void valueChanged(const Eigen::Vector3d& _vector);

public:
  explicit Vector3dEditHorizontal(QWidget* _parent = nullptr);

  Eigen::Vector3d vector() const;
  void setVector(const Eigen::Vector3d& _src);

  void setDecimals(int _decimals);
  void setMinimum(double _minimum);
  void setMaximum(double _maximum);
  void setSingleStep(double _single_step);
  void setSuffix(const QString& _suffix);

  double x() const;
  double y() const;
  double z() const;

private:
  LabeledDoubleSpinBox* x_;
  LabeledDoubleSpinBox* y_;
  LabeledDoubleSpinBox* z_;

private Q_SLOTS:
  void onValueChanged(double _value);
};
}  // namespace qt
}  // namespace tobas
