// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QWidget>
#include <eigen3/Eigen/Core>

#include "./double_spin_box.hpp"

namespace tobas
{
namespace qt
{
class Vector3dEditVertical : public QWidget
{
  Q_OBJECT

  using self = Vector3dEditVertical;
  using super = QWidget;

Q_SIGNALS:
  void valueChanged(const Eigen::Vector3d& vector);

public:
  explicit Vector3dEditVertical(QWidget* parent = nullptr);

  Eigen::Vector3d vector() const;
  void setVector(const Eigen::Vector3d& src);

  void setDecimals(int decimals);
  void setMinimum(double minimum);
  void setMaximum(double maximum);
  void setSingleStep(double single_step);
  void setSuffix(const QString& suffix);

  double x() const;
  double y() const;
  double z() const;

private:
  DoubleSpinBox* x_;
  DoubleSpinBox* y_;
  DoubleSpinBox* z_;

private Q_SLOTS:
  void onValueChanged(double value);
};
}  // namespace qt
}  // namespace tobas
