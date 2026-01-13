#pragma once

#include <QWidget>
#include <eigen3/Eigen/Core>

#include "./labeled_double_spin_box.hpp"

namespace qt
{
class Vector3dEditHorizontal : public QWidget
{
  Q_OBJECT

  using self = Vector3dEditHorizontal;
  using super = QWidget;

Q_SIGNALS:
  void valueChanged(const Eigen::Vector3d& vector);

public:
  explicit Vector3dEditHorizontal(QWidget* parent = nullptr);

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
  LabeledDoubleSpinBox* x_;
  LabeledDoubleSpinBox* y_;
  LabeledDoubleSpinBox* z_;

private Q_SLOTS:
  void onValueChanged(double value);
};
}  // namespace qt
