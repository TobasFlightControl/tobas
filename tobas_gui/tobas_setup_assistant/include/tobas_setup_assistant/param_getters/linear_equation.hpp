#pragma once

#include <tobas_qt_tools/widgets/double_spin_box.hpp>

#include "./base.hpp"

namespace tobas
{
namespace gui
{
namespace sa
{
class ParamGetterWidget_LinearEquation : public ParamGetterWidget<std::pair<double, double>>
{
  Q_OBJECT

  using self = ParamGetterWidget_LinearEquation;
  using super = ParamGetterWidget<ValueType>;

Q_SIGNALS:
  void valueChanged(const ValueType& value);

public:
  explicit ParamGetterWidget_LinearEquation(
    const QString& param_name,
    const QString& left,
    const QString& value,
    const QString& description_text = "");

  ValueType getValue() const override;
  bool setValue(const ValueType& src) override;

  void setDecimals(int decimals);
  void setSuffix(const QString& suffix);

private Q_SLOTS:
  void onValueChanged();

private:
  tobas::qt::DoubleSpinBox* c0_;
  tobas::qt::DoubleSpinBox* c1_;
  QLabel* suffix_;
};
}  // namespace sa
}  // namespace gui
}  // namespace tobas
