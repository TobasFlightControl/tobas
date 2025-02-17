#pragma once

#include <tobas_qt_tools/widgets/spin_box.hpp>

#include "./base.hpp"

namespace gui
{
namespace sa
{
class ParamGetterWidget_SpinBox : public ParamGetterWidget<int>
{
  Q_OBJECT

  using self = ParamGetterWidget_SpinBox;
  using super = ParamGetterWidget<int>;

Q_SIGNALS:
  void valueChanged(int value);

public:
  explicit ParamGetterWidget_SpinBox(const QString& param_name, const QString& description_text);

  int getValue() const override;
  bool setValue(const int& src) override;

  void setMinimum(int minimum);
  void setMaximum(int maximum);
  void setSingleStep(int single_step);
  void setSuffix(const QString& suffix);

private Q_SLOTS:
  void onValueChanged(int value);

private:
  qt::SpinBox* spin_box_;
};
}  // namespace sa
}  // namespace gui
