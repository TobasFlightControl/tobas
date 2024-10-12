#pragma once

#include "./base.hpp"
#include "./scalar_getter.hpp"

namespace gui
{
namespace setup_assistant
{
class ParamGetterWidget_IntRange : public ParamGetterWidget<std::pair<int, int>>
{
  Q_OBJECT

  using self = ParamGetterWidget_IntRange;
  using super = ParamGetterWidget<std::pair<int, int>>;

Q_SIGNALS:
  void valueChanged(std::pair<int, int> value);

public:
  explicit ParamGetterWidget_IntRange(const QString& param_name, const QString& description_text);

  std::pair<int, int> getValue() const override;
  bool setValue(const std::pair<int, int>& src) override;

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
}  // namespace setup_assistant
}  // namespace gui
