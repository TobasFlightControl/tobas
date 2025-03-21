#pragma once

#include <tobas_std_tools/range.hpp>

#include "./base.hpp"
#include "./scalar_getter.hpp"

namespace gui
{
namespace sa
{
class ParamGetterWidget_DoubleRange : public ParamGetterWidget<tobas_std::Range<double>>
{
  Q_OBJECT

  using self = ParamGetterWidget_DoubleRange;
  using super = ParamGetterWidget<ValueType>;

Q_SIGNALS:
  void valueChanged(const ValueType& value);

public:
  explicit ParamGetterWidget_DoubleRange(const QString& param_name, const QString& description_text);

  ValueType getValue() const override;
  bool setValue(const ValueType& src) override;

  void setDecimals(int decimals);
  void setMinimum(double minimum);
  void setMaximum(double maximum);
  void setSingleStep(double single_step);
  void setSuffix(const QString& suffix);

  double min() const;
  double max() const;

  bool isValid() const;

private Q_SLOTS:
  void onValueChanged(double value);

private:
  DoubleGetter* min_;
  DoubleGetter* max_;
};
}  // namespace sa
}  // namespace gui
