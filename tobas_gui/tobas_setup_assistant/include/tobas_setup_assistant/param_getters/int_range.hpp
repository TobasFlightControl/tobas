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
  explicit ParamGetterWidget_IntRange(
    const QString& param_name,
    const QString& description_text = "",
    int minimum = std::numeric_limits<int>::lowest(),
    int maximum = std::numeric_limits<int>::max(),
    int single_step = 1,
  const std::pair<int, int>& _default = {0,0},
    const QString& suffix = "");

  std::pair<int, int> get() const override;
  bool set(const std::pair<int, int>& src) override;

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
