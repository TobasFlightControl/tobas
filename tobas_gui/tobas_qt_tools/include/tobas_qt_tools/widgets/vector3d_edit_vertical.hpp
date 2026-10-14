// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <array>

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
  void valueChanged(const Eigen::Vector3d& _vector);

public:
  explicit Vector3dEditVertical(const QStringList& _labels = {}, QWidget* _parent = nullptr);

  Eigen::Vector3d vector() const;
  void setVector(const Eigen::Vector3d& _src);

  void setDecimals(int _decimals);
  void setMinimum(double _minimum);
  void setMaximum(double _maximum);
  void setRange(double _minimum, double _maximum);
  void setSingleStep(double _single_step);
  void setSuffix(const QString& _suffix);

  double x() const;
  double y() const;
  double z() const;

private:
  std::array<DoubleSpinBox*, 3> spin_boxes_;

private Q_SLOTS:
  void onValueChanged(double _value);
};
}  // namespace qt
}  // namespace tobas
