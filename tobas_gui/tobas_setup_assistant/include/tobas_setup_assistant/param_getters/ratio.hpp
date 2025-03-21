#pragma once

#include <tobas_qt_tools/widgets/spin_box.hpp>

#include "./base.hpp"

namespace gui
{
namespace sa
{
class ParamGetterWidget_Ratio : public ParamGetterWidget<std::pair<int, int>>
{
  Q_OBJECT

  using self = ParamGetterWidget_Ratio;
  using super = ParamGetterWidget<ValueType>;

Q_SIGNALS:
  void valueChanged(const ValueType& value);

public:
  explicit ParamGetterWidget_Ratio(const QString& param_name, const QString& description_text);

  ValueType getValue() const override;
  bool setValue(const ValueType& src) override;

  void setMinimum(int minimum);
  void setMaximum(int maximum);

  void setLeftText(const QString& text);
  void setRightText(const QString& text);

private Q_SLOTS:
  void onValueChanged();

private:
  qt::SpinBox* left_value_;
  qt::SpinBox* right_value_;
  QLabel* left_text_;
  QLabel* right_text_;
};
}  // namespace sa
}  // namespace gui
