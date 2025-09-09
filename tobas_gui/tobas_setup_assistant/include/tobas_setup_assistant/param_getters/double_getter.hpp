#pragma once

#include <tobas_qt_tools/widgets/double_spin_box.hpp>

namespace gui
{
namespace sa
{
class DoubleGetter : public QWidget
{
  Q_OBJECT

  using self = DoubleGetter;
  using super = QWidget;

Q_SIGNALS:
  void valueChanged(double value);

public:
  explicit DoubleGetter(const QString& name);

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
  qt::DoubleSpinBox* data_;
};
}  // namespace sa
}  // namespace gui
