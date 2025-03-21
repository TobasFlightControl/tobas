#pragma once

#include <tobas_std_tools/range.hpp>

#include "./base.hpp"
#include "./scalar_getter.hpp"

namespace gui
{
namespace sa
{
class ParamGetterWidget_IntRange : public ParamGetterWidget<tobas_std::Range<int>>
{
  Q_OBJECT

  using self = ParamGetterWidget_IntRange;
  using super = ParamGetterWidget<ValueType>;

Q_SIGNALS:
  void valueChanged(const ValueType& value);

public:
  explicit ParamGetterWidget_IntRange(const QString& param_name, const QString& description_text);

  ValueType getValue() const override;
  bool setValue(const ValueType& src) override;

  void setMinimum(int minimum);
  void setMaximum(int maximum);
  void setSingleStep(int single_step);
  void setSuffix(const QString& suffix);

  int min() const;
  int max() const;

  bool isValid() const;

private Q_SLOTS:
  void onValueChanged(int value);

private:
  IntGetter* min_;
  IntGetter* max_;
};
}  // namespace sa
}  // namespace gui
