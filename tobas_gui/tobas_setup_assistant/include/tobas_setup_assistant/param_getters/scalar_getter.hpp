#pragma once

#include <tobas_qt_tools/widgets/spin_box.hpp>

namespace gui
{
namespace setup_assistant
{
class IntGetter : public QWidget
{
  Q_OBJECT

  using self = IntGetter;
  using super = QWidget;

Q_SIGNALS:
  void valueChanged(int value);

public:
  explicit IntGetter(const QString& name);

  int getValue() const;
  bool setValue(const int& value);

  void setMinimum(int minimum);
  void setMaximum(int maximum);
  void setSingleStep(int single_step);
  void setSuffix(const QString& suffix);

private Q_SLOTS:
  void onValueChanged(int value);

private:
  qt::SpinBox* data_;
};

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
}  // namespace setup_assistant
}  // namespace gui
