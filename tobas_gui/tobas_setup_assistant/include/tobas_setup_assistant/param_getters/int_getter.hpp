#pragma once

#include <tobas_qt_tools/widgets/spin_box.hpp>

namespace tobas
{
namespace gui
{
namespace sa
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
  tobas::qt::SpinBox* data_;
};
}  // namespace sa
}  // namespace gui
}  // namespace tobas
