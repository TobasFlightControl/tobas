#pragma once

#include <tobas_qt_tools/widgets/double_spin_box.hpp>

namespace tobas
{
namespace qt
{
class LabeledDoubleSpinBox : public QWidget
{
  Q_OBJECT

  using self = LabeledDoubleSpinBox;
  using super = QWidget;

Q_SIGNALS:
  void valueChanged(double value);

public:
  explicit LabeledDoubleSpinBox(const QString& label_text);

  double getValue() const;
  bool setValue(const double& value);

  void setDecimals(int decimals);
  void setMinimum(double minimum);
  void setMaximum(double maximum);
  void setSingleStep(double single_step);
  void setSuffix(const QString& suffix);

private Q_SLOTS:
  void onValueChanged(double value);

private:
  tobas::qt::DoubleSpinBox* data_;
};
}  // namespace qt
}  // namespace tobas
